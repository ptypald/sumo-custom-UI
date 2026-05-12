#!/usr/bin/env python3
import argparse
import json
import logging
import math
import os
import random
import sys
import threading
import time
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import lxml.etree as ET
import numpy as np

TUBE_MPC_DIR = Path(__file__).resolve().parents[1]
if str(TUBE_MPC_DIR) not in sys.path:
    sys.path.append(str(TUBE_MPC_DIR))

# Optional CSAC controller stack from this repository.
try:
    from csac_core import Descriptor  # type: ignore
    from paper_consistent_csac import build_demo_controller  # type: ignore
    from tube_mpc import make_reference, predict_obstacles  # type: ignore
    CSAC_AVAILABLE = True
    CSAC_IMPORT_ERROR = None
except Exception as exc:
    CSAC_AVAILABLE = False
    CSAC_IMPORT_ERROR = exc


if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib  # noqa: E402
import traci  # noqa: E402

try:
    from traci.exceptions import TraCIException  # noqa: E402
except Exception:
    try:
        from traci import TraCIException  # type: ignore  # noqa: E402
    except Exception:
        class TraCIException(Exception):
            pass


APP_DIR = Path(__file__).resolve().parent / "sim"


def parse_sumocfg(cfg_path):
    tree = ET.parse(cfg_path)
    root = tree.getroot()

    def _val(tag):
        el = root.find(f"./input/{tag}")
        return el.get('value') if el is not None else None

    net = _val('net-file')
    rou = _val('route-files')
    gui_el = root.find("./gui_only/gui-settings-file")
    gui = gui_el.get('value') if gui_el is not None else None
    if net is None:
        raise RuntimeError(f"sumocfg missing net-file: {cfg_path}")
    return net, rou, gui


def extract_network_geometry(net):
    edges_out = []
    for edge in net.getEdges():
        if edge.isSpecial():
            continue

        lanes_out = []
        for lane in edge.getLanes():
            shape = lane.getShape()
            if not shape:
                continue
            try:
                width = float(lane.getWidth())
            except Exception:
                width = 3.2

            lanes_out.append({
                "id": lane.getID(),
                "index": lane.getIndex(),
                "speed": float(lane.getSpeed()),
                "length": float(lane.getLength()),
                "width": width,
                "shape": [{"x": float(x), "y": float(y)} for x, y in shape],
            })

        if lanes_out:
            edges_out.append({
                "id": edge.getID(),
                "from": edge.getFromNode().getID() if edge.getFromNode() else None,
                "to": edge.getToNode().getID() if edge.getToNode() else None,
                "function": edge.getFunction(),
                "lanes": lanes_out,
            })

    junctions_out = []
    for node in net.getNodes():
        try:
            x, y = node.getCoord()
            junctions_out.append({
                "id": node.getID(),
                "x": float(x),
                "y": float(y),
                "type": node.getType(),
            })
        except Exception:
            continue

    return {
        "edges": edges_out,
        "junctions": junctions_out,
    }


class SumoRunner:
    def __init__(self, cfg_file, step_length=0.1, sumo_gui=False, client_order=1, lateral_resolution=0.4):
        self.step_length = float(step_length)
        bin_name = "sumo-gui" if sumo_gui else "sumo"
        cmd = [
            bin_name,
            "-c", cfg_file,
            "--step-length", str(self.step_length),
            "--lateral-resolution", str(float(lateral_resolution)),
            "--quit-on-end", "false",
        ]
        traci.start(cmd, label=str(client_order))
        self.conn = traci.getConnection(str(client_order))

    def tick(self):
        self.conn.simulationStep()

    def close(self):
        try:
            self.conn.close(False)
        except Exception:
            pass


class LiveState:
    def __init__(self):
        self.lock = threading.Lock()
        self.running = True
        self.finished = False
        self.stop_requested = False
        self.last_error = None
        self.bounds = None
        self.snapshot = {
            "time": 0.0,
            "step_length": 0.1,
            "vehicles": [],
            "traffic_lights": [],
            "stats": {},
            "network": {"edges": [], "junctions": []},
            "meta": {"running": True, "finished": False, "error": None},
        }

    def update(self, **kwargs):
        with self.lock:
            self.snapshot.update(kwargs)
            self.snapshot["meta"] = {
                "running": self.running,
                "finished": self.finished,
                "error": self.last_error,
            }
            if self.bounds is not None:
                self.snapshot["bounds"] = self.bounds

    def get_snapshot(self):
        with self.lock:
            return json.loads(json.dumps(self.snapshot))

    def set_running(self, value):
        with self.lock:
            self.running = bool(value)
            self.snapshot["meta"] = {
                "running": self.running,
                "finished": self.finished,
                "error": self.last_error,
            }

    def set_finished(self, value=True):
        with self.lock:
            self.finished = bool(value)
            self.snapshot["meta"] = {
                "running": self.running,
                "finished": self.finished,
                "error": self.last_error,
            }

    def set_error(self, message):
        with self.lock:
            self.last_error = message
            self.snapshot["meta"] = {
                "running": self.running,
                "finished": self.finished,
                "error": self.last_error,
            }


class SimulationService:
    def __init__(self, args, state):
        self.args = args
        self.state = state
        self.runner = None
        self.conn = None
        self.net = None
        self.network_geometry = {"edges": [], "junctions": []}
        self.thread = None
        self.ego_vehicles = set()
        self.csac_agents = {}
        self.csac_metrics = {
            "calls": 0,
            "success": 0,
            "failures": 0,
            "accepted": 0,
            "fallback_used": 0,
            "total_ms": 0.0,
            "max_ms": 0.0,
            "last_ms": 0.0,
            "last_ax": 0.0,
            "last_ay": 0.0,
            "last_vehicle": None,
            "last_error": None,
        }
        self._next_csac_profile_log_s = 0.0
        self.random = random.Random(args.random_seed)
        self.cfg_file = Path(args.sumo_cfg).resolve()
        self.net_file = None
        self.csac_scenario_name = str(args.csac_scenario)

    def _resolve_csac_scenario_name(self):
        if str(self.args.csac_scenario).lower() != "auto":
            return str(self.args.csac_scenario)
        max_lanes = 0
        try:
            for edge in self.net.getEdges():
                if edge.isSpecial():
                    continue
                max_lanes = max(max_lanes, len(edge.getLanes()))
        except Exception:
            max_lanes = 0
        return "merge_two_main_lanes" if max_lanes >= 3 else "merge_one_main_lane"

    def build(self):
        if not CSAC_AVAILABLE:
            raise RuntimeError(f"CSAC controller imports failed: {CSAC_IMPORT_ERROR}")

        net_file, _, _ = parse_sumocfg(str(self.cfg_file))
        net_path = Path(net_file)
        if not net_path.is_absolute():
            net_path = self.cfg_file.parent / net_path
        self.net_file = net_path.resolve()

        self.net = sumolib.net.readNet(str(self.net_file))
        self.network_geometry = extract_network_geometry(self.net)
        self.csac_scenario_name = self._resolve_csac_scenario_name()
        logging.info("CSAC scenario resolved to '%s' from SUMO network", self.csac_scenario_name)

        xmin, ymin, xmax, ymax = self.net.getBoundary()
        self.state.bounds = {
            "xmin": xmin,
            "ymin": ymin,
            "xmax": xmax,
            "ymax": ymax,
        }

        self.runner = SumoRunner(
            str(self.cfg_file),
            step_length=self.args.step_length,
            sumo_gui=self.args.sumo_gui,
            client_order=self.args.client_order,
            lateral_resolution=self.args.lateral_resolution,
        )
        self.conn = self.runner.conn

        if self.args.sumo_gui and self.args.view_schema:
            try:
                self.conn.gui.setSchema(self.conn.gui.DEFAULT_VIEW, self.args.view_schema)
            except Exception as exc:
                logging.warning("Could not set SUMO GUI schema %s: %s", self.args.view_schema, exc)

        self.state.update(
            step_length=self.args.step_length,
            network=self.network_geometry,
        )

    def start(self):
        self.build()
        self.build_snapshot()
        self.thread = threading.Thread(target=self.run_loop, daemon=True)
        self.thread.start()

    def select_ego(self, new_vehicle_ids):
        explicit_ids = set(self.args.ego_ids or [])
        for vid in new_vehicle_ids:
            if explicit_ids:
                pick_as_ego = vid in explicit_ids
            else:
                pick_as_ego = self.random.random() < float(self.args.ego_penetration)
            if pick_as_ego:
                self.ego_vehicles.add(vid)
                try:
                    self.conn.vehicle.setColor(vid, (255, 0, 0, 255))
                    # Keep SUMO safety checks for speed control while accepting
                    # externally-driven acceleration targets.
                    self.conn.vehicle.setSpeedMode(vid, 31)
                    # Disable autonomous lane changes; permit TraCI sublane control.
                    # bit 5 (32) = respond to changeSublane, bit 6 (64) = override safety.
                    # self.conn.vehicle.setLaneChangeMode(vid, 32 + 64)
                    self.conn.vehicle.setLaneChangeMode(vid, 512)
                    # Allow lateral movement up to 2.5 m/s for lane changes.
                    self.conn.vehicle.setMaxSpeedLat(vid, 2.5)
                    self.conn.vehicle.setMaxSpeed(vid, float(self.args.csac_target_speed))
                except Exception:
                    pass
            else:
                try:
                    self.conn.vehicle.setColor(vid, (37, 99, 235, 255))
                    # Keep non-ego SUMO vehicles in their current lane so
                    # lane-change behavior comes only from controlled egos.
                    self.conn.vehicle.setLaneChangeMode(vid, 0)
                    self.conn.vehicle.setMaxSpeed(vid, float(self.args.sumo_target_speed))
                except Exception:
                    pass

    @staticmethod
    def _unit_heading_from_shape(shape):
        if not shape or len(shape) < 2:
            return np.array([1.0, 0.0], dtype=float)
        dx = float(shape[-1][0] - shape[0][0])
        dy = float(shape[-1][1] - shape[0][1])
        n = math.hypot(dx, dy)
        if n <= 1e-9:
            return np.array([1.0, 0.0], dtype=float)
        return np.array([dx / n, dy / n], dtype=float)

    def _vehicle_state_xy(self, veh_id):
        x, y = self.conn.vehicle.getPosition(veh_id)
        v = float(self.conn.vehicle.getSpeed(veh_id))
        lane_id = self.conn.vehicle.getLaneID(veh_id)
        shape = self.conn.lane.getShape(lane_id)
        heading = self._unit_heading_from_shape(shape)
        vx = float(v * heading[0])
        try:
            vy = float(self.conn.vehicle.getLateralSpeed(veh_id))
        except Exception:
            vy = 0.0
        return np.array([float(x), float(y), vx, vy], dtype=float), lane_id

    def _lane_center_y(self, lane_id):
        shape = self.conn.lane.getShape(lane_id)
        if not shape:
            return None
        return float(shape[0][1])

    def _edge_lane_centers(self, edge_id):
        try:
            n_lanes = int(self.conn.edge.getLaneNumber(edge_id))
        except Exception:
            return []
        out = []
        for idx in range(n_lanes):
            lane_id = f"{edge_id}_{idx}"
            try:
                y = self._lane_center_y(lane_id)
                if y is None:
                    continue
                out.append((idx, y))
            except Exception:
                continue
        return out

    def _edge_lane_geometry(self, edge_id):
        centers = self._edge_lane_centers(edge_id)
        if not centers:
            return None
        ys = sorted(float(y) for _, y in centers)
        y_center = float(np.mean(ys))
        if len(ys) >= 2:
            lane_width_world = float(np.median(np.abs(np.diff(np.asarray(ys, dtype=float)))))
        else:
            lane_idx = int(centers[0][0])
            lane_id = f"{edge_id}_{lane_idx}"
            try:
                lane_width_world = float(self.conn.lane.getWidth(lane_id))
            except Exception:
                lane_width_world = 3.2
        lane_width_world = max(abs(lane_width_world), 0.5)
        return {
            "edge_id": str(edge_id),
            "lane_centers_world": tuple(ys),
            "lane_centers_zeroed": tuple(float(y - y_center) for y in ys),
            "y_center": y_center,
            "lane_width_world": lane_width_world,
        }

    def _edge_lateral_frame(self, edge_id, lane_width_ref):
        geom = self._edge_lane_geometry(edge_id)
        if geom is None:
            return None
        lane_width_world = float(geom["lane_width_world"])
        scale = float(lane_width_ref / max(lane_width_world, 1e-6))
        return {
            "y_center": float(geom["y_center"]),
            "lane_width_world": lane_width_world,
            "scale": scale,
        }

    def _agent_lateral_frame(self, veh_id, lane_id, agent):
        lane_width_ref = float(agent.get("lane_width_ref", 3.2))
        frame = None
        edge_id = self.conn.vehicle.getRoadID(veh_id)
        if edge_id and not str(edge_id).startswith(":"):
            frame = self._edge_lateral_frame(edge_id, lane_width_ref)
        if frame is None and lane_id:
            edge_from_lane = str(lane_id).rsplit("_", 1)[0]
            frame = self._edge_lateral_frame(edge_from_lane, lane_width_ref)
        if frame is None:
            frame = agent.get("lateral_frame")
        if frame is None:
            frame = {"y_center": 0.0, "lane_width_world": lane_width_ref, "scale": 1.0}
        agent["lateral_frame"] = frame
        return frame

    @staticmethod
    def _sumo_to_csac_lateral(y_world, vy_world, frame):
        scale = float(frame.get("scale", 1.0))
        y_center = float(frame.get("y_center", 0.0))
        return float((y_world - y_center) * scale), float(vy_world * scale)

    @staticmethod
    def _csac_to_sumo_lateral(y_csac, vy_csac, frame):
        scale = float(frame.get("scale", 1.0))
        y_center = float(frame.get("y_center", 0.0))
        return float(y_center + y_csac / max(scale, 1e-6)), float(vy_csac / max(scale, 1e-6))

    def _ensure_csac_agent(self, veh_id):
        agent = self.csac_agents.get(veh_id)
        if agent is not None:
            return agent
        lane_centers_ref = None
        lane_width_ref = None
        edge_id = None
        try:
            edge_id = self.conn.vehicle.getRoadID(veh_id)
            if edge_id and str(edge_id).startswith(":"):
                edge_id = None
        except Exception:
            edge_id = None
        if edge_id is None:
            try:
                lane_id = self.conn.vehicle.getLaneID(veh_id)
                edge_id = str(lane_id).rsplit("_", 1)[0] if lane_id else None
            except Exception:
                edge_id = None
        if edge_id is not None:
            geom = self._edge_lane_geometry(edge_id)
            if geom is not None:
                lane_centers_ref = tuple(float(v) for v in geom["lane_centers_zeroed"])
                lane_width_ref = float(geom["lane_width_world"])

        cfg, _, controller, scenario = build_demo_controller(
            scenario_name=self.csac_scenario_name,
            lane_centers=lane_centers_ref,
            lane_width=lane_width_ref,
        )
        if lane_width_ref is None:
            centers_ref = tuple(float(v) for v in getattr(scenario.cfg, "lane_centers", tuple()))
            if len(centers_ref) >= 2:
                lane_width_ref = float(np.median(np.diff(np.sort(np.asarray(centers_ref, dtype=float)))))
                lane_width_ref = max(abs(lane_width_ref), 0.5)
            else:
                lane_width_ref = 3.2
        agent = {
            "cfg": cfg,
            "controller": controller,
            "lat_v": 0.0,
            "target_y": None,
            "csac_plan": None,
            "csac_plan_t0": None,
            "csac_plan_dt": float(cfg.dt),
            "lane_width_ref": float(lane_width_ref),
            "lateral_frame": None,
        }
        self.csac_agents[veh_id] = agent
        return agent

    def _record_csac_metrics(self, veh_id, elapsed_ms, u=None, accepted=False, fallback_used=False, error=None):
        m = self.csac_metrics
        m["calls"] = int(m["calls"]) + 1
        m["total_ms"] = float(m["total_ms"]) + float(elapsed_ms)
        m["max_ms"] = max(float(m["max_ms"]), float(elapsed_ms))
        m["last_ms"] = float(elapsed_ms)
        m["last_vehicle"] = str(veh_id)
        if u is not None:
            m["last_ax"] = float(u[0])
            m["last_ay"] = float(u[1])
        if accepted:
            m["accepted"] = int(m["accepted"]) + 1
        if fallback_used:
            m["fallback_used"] = int(m["fallback_used"]) + 1
        if error is None:
            m["success"] = int(m["success"]) + 1
            m["last_error"] = None
        else:
            m["failures"] = int(m["failures"]) + 1
            m["last_error"] = str(error)

        if self.args.debug:
            try:
                sim_t = float(self.conn.simulation.getTime())
            except Exception:
                sim_t = 0.0
            if sim_t >= self._next_csac_profile_log_s:
                avg_ms = float(m["total_ms"]) / max(int(m["calls"]), 1)
                logging.info(
                    "CSAC profile t=%.1fs calls=%d ok=%d fail=%d avg=%.2fms max=%.2fms last=%.2fms",
                    sim_t,
                    int(m["calls"]),
                    int(m["success"]),
                    int(m["failures"]),
                    avg_ms,
                    float(m["max_ms"]),
                    float(m["last_ms"]),
                )
                self._next_csac_profile_log_s = sim_t + 2.0

    def _lane_leader_stats(self, veh_id, edge_id, lane_idx, x_ego):
        lead_gap = np.inf
        lead_speed = float(self.args.csac_target_speed)
        try:
            for other_id in self.conn.vehicle.getIDList():
                if other_id == veh_id:
                    continue
                if self.conn.vehicle.getRoadID(other_id) != edge_id:
                    continue
                if int(self.conn.vehicle.getLaneIndex(other_id)) != int(lane_idx):
                    continue
                ox, _ = self.conn.vehicle.getPosition(other_id)
                gap = float(ox - x_ego)
                if gap <= 0.0 or gap >= lead_gap:
                    continue
                lead_gap = gap
                lead_speed = float(self.conn.vehicle.getSpeed(other_id))
        except Exception:
            pass
        return float(lead_gap), float(lead_speed)

    def _select_soft_target_y(self, veh_id, x_world):
        """Select a soft lateral reference from network geometry (not lane state machine)."""
        try:
            edge_id = self.conn.vehicle.getRoadID(veh_id)
            if not edge_id or edge_id.startswith(":"):
                return float(x_world[1])
            centers = self._edge_lane_centers(edge_id)
            if not centers:
                return float(x_world[1])

            agent = self._ensure_csac_agent(veh_id)
            prev_target_y = agent.get("target_y")
            if prev_target_y is None:
                prev_target_y = float(x_world[1])

            x_ego = float(x_world[0])
            y_ego = float(x_world[1])
            target_v = float(self.args.csac_target_speed)

            best_y = float(prev_target_y)
            best_score = -1e9
            for lane_idx, lane_y in centers:
                gap, lead_v = self._lane_leader_stats(veh_id, edge_id, lane_idx, x_ego)
                gap_term = 120.0 if not np.isfinite(gap) else float(np.clip(gap, 0.0, 120.0))
                speed_term = float(np.clip(lead_v, 0.0, target_v))
                comfort_pen = 0.35 * abs(float(lane_y) - y_ego)
                hysteresis_pen = 0.10 * abs(float(lane_y) - float(prev_target_y))
                score = speed_term + 0.05 * gap_term - comfort_pen - hysteresis_pen
                if score > best_score:
                    best_score = score
                    best_y = float(lane_y)

            alpha = float(np.clip(self.args.step_length / 1.0, 0.06, 0.30))
            y_ref = (1.0 - alpha) * float(prev_target_y) + alpha * float(best_y)
            return float(y_ref)
        except Exception:
            return float(x_world[1])

    def _nearby_obstacles_csac(self, veh_id, frame, horizon, dt, max_obs=4, scan_radius=80.0):
        obs_states = []
        obs_names = []
        try:
            x_ego, y_ego = self.conn.vehicle.getPosition(veh_id)
            for other_id in self.conn.vehicle.getIDList():
                if other_id == veh_id:
                    continue
                ox, oy = self.conn.vehicle.getPosition(other_id)
                if abs(ox - x_ego) > scan_radius or abs(oy - y_ego) > scan_radius:
                    continue
                ov = float(self.conn.vehicle.getSpeed(other_id))
                other_lane = self.conn.vehicle.getLaneID(other_id)
                other_shape = self.conn.lane.getShape(other_lane)
                heading = self._unit_heading_from_shape(other_shape)
                ovx = float(ov * heading[0])
                try:
                    ovy = float(self.conn.vehicle.getLateralSpeed(other_id))
                except Exception:
                    ovy = 0.0
                oy_csac, ovy_csac = self._sumo_to_csac_lateral(float(oy), float(ovy), frame)
                obs_states.append([float(ox), oy_csac, ovx, ovy_csac])
                obs_names.append(other_id)
                if len(obs_states) >= int(max_obs):
                    break
        except Exception:
            pass
        if not obs_states:
            return np.zeros((0, horizon + 1, 2), dtype=float), []
        return predict_obstacles(np.asarray(obs_states, dtype=float), int(horizon), float(dt)), obs_names

    def ego_csac(self, veh_id, dt):
        t0 = time.perf_counter()
        try:
            agent = self._ensure_csac_agent(veh_id)
            cfg = agent["cfg"]
            controller = agent["controller"]

            x_world, lane_id = self._vehicle_state_xy(veh_id)
            frame = self._agent_lateral_frame(veh_id, lane_id, agent)
            y_csac, vy_csac = self._sumo_to_csac_lateral(float(x_world[1]), float(x_world[3]), frame)
            xk_local = np.array([float(x_world[0]), y_csac, float(x_world[2]), vy_csac], dtype=float)

            target_y_world = self._select_soft_target_y(veh_id, x_world)
            agent["target_y"] = float(target_y_world)
            target_y_local, _ = self._sumo_to_csac_lateral(float(target_y_world), 0.0, frame)
            lane_limit = float(self.conn.lane.getMaxSpeed(lane_id))
            target_vx = float(min(self.args.csac_target_speed, lane_limit * self.args.csac_speed_limit_scale))
            z_ref = make_reference(xk_local, controller.N, cfg.dt, target_y=target_y_local, target_vx=target_vx)

            obs_pred, obs_names = self._nearby_obstacles_csac(
                veh_id, frame, controller.N, cfg.dt,
                max_obs=int(cfg.max_obs),
            )

            z_truck = 0.0
            z_inc = 0.0
            leader_info = self.conn.vehicle.getLeader(veh_id, 250.0)
            if leader_info is not None:
                leader_id, gap = leader_info
                leader_type = str(self.conn.vehicle.getTypeID(leader_id)).lower()
                z_truck = 1.0 if any(token in leader_type for token in ("truck", "bus", "trailer", "heavy")) else 0.0
                z_inc = float(np.clip((45.0 - float(gap)) / 35.0, 0.0, 1.0))

            z_hat = Descriptor(
                z_truck=float(z_truck),
                z_vis=0.05,
                z_inc=float(z_inc),
                z_const=0,
            )
            delta_k = Descriptor(z_truck=0.10, z_vis=0.10, z_inc=0.10, z_const=0)
            confidence = 0.95 if leader_info is None else 0.85

            result = controller.step(
                xk=xk_local,
                z_hat=z_hat,
                delta_k=delta_k,
                confidence=confidence,
                z_ref=z_ref,
                obs_pred=obs_pred,
                obs_names=obs_names,
            )

            speed_now = float(self.conn.vehicle.getSpeed(veh_id))
            speed_next = max(0.0, speed_now + float(result.u[0]) * float(dt))
            speed_next = min(speed_next, lane_limit)
            speed_next = min(speed_next, float(self.args.csac_target_speed))
            scale = float(frame.get("scale", 1.0))
            ay_world = float(result.u[1]) / max(scale, 1e-6)
            nominal = getattr(controller, "_prev_nominal_x", None)
            if isinstance(nominal, np.ndarray) and nominal.ndim == 2 and nominal.shape[1] >= 4:
                plan_csac = np.asarray(nominal[:, :4], dtype=float).copy()
                plan_world = plan_csac.copy()
                for i in range(plan_world.shape[0]):
                    y_w, vy_w = self._csac_to_sumo_lateral(float(plan_csac[i, 1]), float(plan_csac[i, 3]), frame)
                    plan_world[i, 1] = y_w
                    plan_world[i, 3] = vy_w
                agent["csac_plan"] = plan_world
                agent["csac_plan_dt"] = float(cfg.dt)
                try:
                    agent["csac_plan_t0"] = float(self.conn.simulation.getTime())
                except Exception:
                    agent["csac_plan_t0"] = None
            elapsed_ms = (time.perf_counter() - t0) * 1000.0
            self._record_csac_metrics(
                veh_id=veh_id,
                elapsed_ms=elapsed_ms,
                u=np.array([float(result.u[0]), ay_world], dtype=float),
                accepted=bool(getattr(result, "accepted_by_filter", False)),
                fallback_used=bool(getattr(result, "fallback_used", False)),
                error=None,
            )
            return float(speed_next), float(ay_world)
        except Exception as exc:
            elapsed_ms = (time.perf_counter() - t0) * 1000.0
            self._record_csac_metrics(
                veh_id=veh_id,
                elapsed_ms=elapsed_ms,
                u=None,
                accepted=False,
                fallback_used=False,
                error=exc,
            )
            if self.args.debug:
                logging.exception("CSAC step failed for %s", veh_id)
            try:
                return float(self.conn.vehicle.getSpeed(veh_id)), 0.0
            except Exception:
                return 0.0, 0.0

    def _apply_csac_lateral(self, veh_id, ay_cmd, dt):
        try:
            agent = self._ensure_csac_agent(veh_id)
            plan = agent.get("csac_plan")
            plan_t0 = agent.get("csac_plan_t0")
            plan_dt = float(agent.get("csac_plan_dt", dt))
            lat_shift = None
            if isinstance(plan, np.ndarray) and plan.ndim == 2 and plan.shape[0] >= 2 and plan.shape[1] >= 2:
                y_now = float(self.conn.vehicle.getPosition(veh_id)[1])
                if plan_t0 is not None and plan_dt > 1e-6:
                    now_t = float(self.conn.simulation.getTime())
                    idx = int(np.floor((now_t - float(plan_t0)) / plan_dt)) + 1
                    idx = int(np.clip(idx, 1, plan.shape[0] - 1))
                else:
                    idx = 1
                y_des = float(plan[idx, 1])
                lat_shift = float(np.clip(y_des - y_now, -0.8, 0.8))
                agent["lat_v"] = float(lat_shift / max(float(dt), 1e-6))
            if lat_shift is None:
                lat_v = float(agent.get("lat_v", 0.0))
                lat_v = float(np.clip(lat_v + float(ay_cmd) * float(dt), -2.0, 2.0))
                agent["lat_v"] = lat_v
                lat_shift = float(np.clip(lat_v * float(dt), -0.6, 0.6))
            self.conn.vehicle.changeSublane(veh_id, lat_shift)
        except Exception:
            pass

    def _apply_ego_controller(self):
        veh_ids = set(self.conn.vehicle.getIDList())
        for vid in (self.ego_vehicles & veh_ids):
            new_speed, ay_cmd = self.ego_csac(vid, dt=self.args.step_length)
            try:
                self.conn.vehicle.setSpeed(vid, new_speed)
            except Exception:
                pass
            self._apply_csac_lateral(vid, ay_cmd=ay_cmd, dt=self.args.step_length)

    def build_snapshot(self):
        sim_time = self.conn.simulation.getTime()
        vehicle_ids = list(self.conn.vehicle.getIDList())
        vehicles = []

        for vid in vehicle_ids:
            try:
                x, y = self.conn.vehicle.getPosition(vid)
                rgba = self.conn.vehicle.getColor(vid)
                try:
                    x_state, _ = self._vehicle_state_xy(vid)
                    vx = float(x_state[2])
                    vy = float(x_state[3])
                except Exception:
                    speed = float(self.conn.vehicle.getSpeed(vid))
                    angle_deg = float(self.conn.vehicle.getAngle(vid))
                    angle_rad = math.radians(90.0 - angle_deg)
                    vx = float(speed * math.cos(angle_rad))
                    vy = 0.0
                csac_plan = None
                if vid in self.ego_vehicles:
                    agent = self.csac_agents.get(vid)
                    if agent is not None:
                        plan = agent.get("csac_plan")
                        if isinstance(plan, np.ndarray) and plan.ndim == 2 and plan.shape[1] >= 4 and plan.shape[0] >= 1:
                            dt_plan = float(agent.get("csac_plan_dt", self.args.step_length))
                            t0_plan = agent.get("csac_plan_t0")
                            t0_plan = None if t0_plan is None else float(t0_plan)
                            csac_plan = {
                                "t0": t0_plan,
                                "dt": round(dt_plan, 4),
                                "states": [
                                    {
                                        "x": round(float(p[0]), 3),
                                        "y": round(float(p[1]), 3),
                                        "vx": round(float(p[2]), 3),
                                        "vy": round(float(p[3]), 3),
                                    }
                                    for p in np.asarray(plan, dtype=float)
                                ],
                            }
                vehicles.append({
                    "id": vid,
                    "x": round(float(x), 3),
                    "y": round(float(y), 3),
                    "angle": round(float(self.conn.vehicle.getAngle(vid)), 3),
                    "speed": round(float(self.conn.vehicle.getSpeed(vid)), 3),
                    "vx": round(vx, 3),
                    "vy": round(vy, 3),
                    "lane_id": self.conn.vehicle.getLaneID(vid),
                    "road_id": self.conn.vehicle.getRoadID(vid),
                    "type_id": self.conn.vehicle.getTypeID(vid),
                    "length": round(float(self.conn.vehicle.getLength(vid)), 3),
                    "width": round(float(self.conn.vehicle.getWidth(vid)), 3),
                    "is_ego": vid in self.ego_vehicles,
                    "color": list(rgba),
                    "csac_plan": csac_plan,
                })
            except TraCIException:
                continue

        traffic_lights = []
        for tid in self.conn.trafficlight.getIDList():
            try:
                state = self.conn.trafficlight.getRedYellowGreenState(tid)
                phase = self.conn.trafficlight.getPhase(tid)
                next_switch = self.conn.trafficlight.getNextSwitch(tid)

                try:
                    x, y = self.conn.junction.getPosition(tid)
                except Exception:
                    x, y = (0.0, 0.0)

                traffic_lights.append({
                    "id": tid,
                    "state": state,
                    "phase": phase,
                    "time_to_switch": round(max(0.0, next_switch - sim_time), 3),
                    "x": round(float(x), 3),
                    "y": round(float(y), 3),
                })
            except Exception:
                continue

        self.state.update(
            time=round(float(sim_time), 3),
            vehicles=vehicles,
            traffic_lights=traffic_lights,
            stats={
                "vehicle_count": len(vehicles),
                "ego_count": len(self.ego_vehicles & set(vehicle_ids)),
                "mean_speed": round(sum(v["speed"] for v in vehicles) / len(vehicles), 3) if vehicles else 0.0,
                "csac": {
                    "enabled": True,
                    "calls": int(self.csac_metrics["calls"]),
                    "success": int(self.csac_metrics["success"]),
                    "failures": int(self.csac_metrics["failures"]),
                    "accepted": int(self.csac_metrics["accepted"]),
                    "fallback_used": int(self.csac_metrics["fallback_used"]),
                    "avg_ms": round(float(self.csac_metrics["total_ms"]) / max(int(self.csac_metrics["calls"]), 1), 3),
                    "max_ms": round(float(self.csac_metrics["max_ms"]), 3),
                    "last_ms": round(float(self.csac_metrics["last_ms"]), 3),
                    "last_ax": round(float(self.csac_metrics["last_ax"]), 4),
                    "last_ay": round(float(self.csac_metrics["last_ay"]), 4),
                    "last_vehicle": self.csac_metrics["last_vehicle"],
                    "last_error": self.csac_metrics["last_error"],
                },
            },
        )

    def run_loop(self):
        try:
            while not self.state.stop_requested and self.conn.simulation.getTime() < self.args.sim_time:
                if not self.state.running:
                    time.sleep(0.05)
                    continue

                loop_start = time.time()
                self.runner.tick()

                entered = list(self.conn.simulation.getDepartedIDList())
                if entered:
                    self.select_ego(entered)

                exited = list(self.conn.simulation.getArrivedIDList())
                for vid in exited:
                    self.ego_vehicles.discard(vid)
                    self.csac_agents.pop(vid, None)

                self._apply_ego_controller()

                self.build_snapshot()

                if self.args.realtime:
                    elapsed = time.time() - loop_start
                    if elapsed < self.args.step_length:
                        time.sleep(self.args.step_length - elapsed)

        except Exception as exc:
            logging.exception("Simulation loop failed")
            self.state.set_error(str(exc))
        finally:
            self.state.set_finished(True)
            try:
                self.runner.close()
            except Exception:
                pass

    def step_once(self):
        if self.state.finished:
            return
        self.runner.tick()
        entered = list(self.conn.simulation.getDepartedIDList())
        if entered:
            self.select_ego(entered)
        exited = list(self.conn.simulation.getArrivedIDList())
        for vid in exited:
            self.ego_vehicles.discard(vid)
            self.csac_agents.pop(vid, None)
        self._apply_ego_controller()
        self.build_snapshot()


class AppHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, directory=None, state=None, service=None, **kwargs):
        self._state = state
        self._service = service
        super().__init__(*args, directory=directory, **kwargs)

    def end_json(self, payload, status=HTTPStatus.OK):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/api/state":
            self.end_json(self._state.get_snapshot())
            return
        if parsed.path == "/api/health":
            self.end_json({"ok": True, "meta": self._state.get_snapshot().get("meta", {})})
            return
        if parsed.path == "/":
            # Default to the stable vanilla-ESM visualizer.
            self.path = "/visualize.html"
        return super().do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path != "/api/control":
            self.send_error(HTTPStatus.NOT_FOUND, "Unknown endpoint")
            return

        length = int(self.headers.get("Content-Length", 0) or 0)
        raw = self.rfile.read(length) if length else b""
        try:
            payload = json.loads(raw.decode("utf-8") or "{}")
        except json.JSONDecodeError:
            payload = parse_qs(raw.decode("utf-8"))

        action = payload.get("action")
        if isinstance(action, list):
            action = action[0]

        if action == "pause":
            self._state.set_running(False)
        elif action == "resume":
            self._state.set_running(True)
        elif action == "toggle":
            self._state.set_running(not self._state.running)
        elif action == "step":
            self._state.set_running(False)
            self._service.step_once()
        else:
            self.end_json({"ok": False, "error": "Unsupported action"}, status=HTTPStatus.BAD_REQUEST)
            return

        self.end_json({"ok": True, "meta": self._state.get_snapshot().get("meta", {})})

    def log_message(self, fmt, *args):
        # Silence per-request HTTP access logs (e.g. /api/state polling).
        return


def build_handler(directory, state, service):
    def _handler(*args, **kwargs):
        return AppHandler(*args, directory=directory, state=state, service=service, **kwargs)
    return _handler


def main(args):
    logging.basicConfig(
        format='%(levelname)s: %(message)s',
        level=logging.DEBUG if args.debug else logging.INFO,
    )

    state = LiveState()
    service = SimulationService(args, state)
    service.start()

    handler = build_handler(str(APP_DIR), state, service)
    httpd = ThreadingHTTPServer((args.http_host, args.http_port), handler)
    logging.info("Viewer available at http://%s:%s", args.http_host, args.http_port)

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        logging.info("Stopping...")
    finally:
        state.stop_requested = True
        httpd.shutdown()
        httpd.server_close()
        if service.thread is not None:
            service.thread.join(timeout=2.0)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Run SUMO and serve a live HTML visualizer")
    parser.add_argument('--sumo-cfg', default='networks/highway/highway.sumocfg', help='Path to .sumocfg')
    parser.add_argument('--sumo-gui', action='store_true', default=False, help='Run SUMO with GUI too')
    parser.add_argument('--step-length', type=float, default=0.1, help='Simulation step length [s]')
    parser.add_argument('--sim-time', type=float, default=600.0, help='Total simulation time [s]')
    parser.add_argument('--client-order', type=int, default=1, help='TraCI client order')
    parser.add_argument('--view-schema', default='real world', help='SUMO-GUI color scheme')
    # Backward-compat flag: controller selection is CSAC-only now.
    parser.add_argument('--ego-controller', choices=('csac',), default='csac', help=argparse.SUPPRESS)
    parser.add_argument('--ego-ids', nargs='*', default=None, help='Explicit vehicle IDs to control as ego')
    parser.add_argument('--ego-penetration', type=float, default=1.0)
    parser.add_argument('--random-seed', type=int, default=42)
    parser.add_argument(
        '--csac-scenario',
        default='auto',
        help="CSAC scenario template or 'auto' to resolve from SUMO network lane count",
    )
    parser.add_argument('--csac-target-speed', type=float, default=25.0, help='CSAC cruise target speed [m/s]')
    parser.add_argument('--sumo-target-speed', type=float, default=20.0, help='Non-ego SUMO target speed cap [m/s]')
    parser.add_argument('--csac-speed-limit-scale', type=float, default=1.0, help='Scale factor on lane speed limit')
    parser.add_argument('--lateral-resolution', type=float, default=0.4, help='SUMO lateral resolution for sublane motion [m]')
    parser.add_argument('--http-host', default='127.0.0.1', help='HTTP bind host')
    parser.add_argument('--http-port', type=int, default=8000, help='HTTP bind port')
    parser.add_argument('--realtime', action='store_true', default=True, help='Throttle to wall-clock time')
    parser.add_argument('--debug', action='store_true')
    args = parser.parse_args()
    main(args)
