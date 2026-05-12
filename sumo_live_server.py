#!/usr/bin/env python3

import argparse
import json
import logging
import math
import os
import sys
import threading
import time
from http import HTTPStatus
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import lxml.etree as ET
import numpy as np

if "SUMO_HOME" in os.environ:
    sys.path.append(os.path.join(os.environ["SUMO_HOME"], "tools"))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib  # noqa: E402
import traci  # noqa: E402

try:
    from traci.exceptions import TraCIException  # noqa: E402
except Exception:
    try:
        from traci import TraCIException  # type: ignore # noqa: E402
    except Exception:
        class TraCIException(Exception):
            pass


APP_DIR = Path(__file__).resolve().parent / "sim"


def parse_sumocfg(cfg_path):
    tree = ET.parse(cfg_path)
    root = tree.getroot()

    def _val(tag):
        el = root.find(f"./input/{tag}")
        return el.get("value") if el is not None else None

    net = _val("net-file")
    rou = _val("route-files")
    gui_el = root.find("./gui_only/gui-settings-file")
    gui = gui_el.get("value") if gui_el is not None else None

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

        self.cfg_file = Path(args.sumo_cfg).resolve()
        self.net_file = None

    def build(self):
        net_file, _, _ = parse_sumocfg(str(self.cfg_file))
        net_path = Path(net_file)

        if not net_path.is_absolute():
            net_path = self.cfg_file.parent / net_path

        self.net_file = net_path.resolve()
        self.net = sumolib.net.readNet(str(self.net_file))
        self.network_geometry = extract_network_geometry(self.net)

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

    def style_new_vehicles(self, new_vehicle_ids):
        """SUMO-only mode.

        Vehicles are controlled entirely by SUMO. This function only colors
        newly departed vehicles for visualization and does not override speed,
        acceleration, lane changing, or sublane motion.
        """
        for vid in new_vehicle_ids:
            try:
                self.conn.vehicle.setColor(vid, (37, 99, 235, 255))
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

        try:
            shape = self.conn.lane.getShape(lane_id)
            heading = self._unit_heading_from_shape(shape)
            vx = float(v * heading[0])
        except Exception:
            angle_deg = float(self.conn.vehicle.getAngle(veh_id))
            angle_rad = math.radians(90.0 - angle_deg)
            vx = float(v * math.cos(angle_rad))

        try:
            vy = float(self.conn.vehicle.getLateralSpeed(veh_id))
        except Exception:
            vy = 0.0

        return np.array([float(x), float(y), vx, vy], dtype=float), lane_id

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
                    "is_ego": False,
                    "color": list(rgba),
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
                    x, y = 0.0, 0.0

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

        mean_speed = round(sum(v["speed"] for v in vehicles) / len(vehicles), 3) if vehicles else 0.0

        self.state.update(
            time=round(float(sim_time), 3),
            vehicles=vehicles,
            traffic_lights=traffic_lights,
            stats={
                "vehicle_count": len(vehicles),
                "mean_speed": mean_speed,
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
                    self.style_new_vehicles(entered)

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
            self.style_new_vehicles(entered)

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
            self.end_json({
                "ok": True,
                "meta": self._state.get_snapshot().get("meta", {}),
            })
            return

        if parsed.path == "/":
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
            self.end_json(
                {"ok": False, "error": "Unsupported action"},
                status=HTTPStatus.BAD_REQUEST,
            )
            return

        self.end_json({
            "ok": True,
            "meta": self._state.get_snapshot().get("meta", {}),
        })

    def log_message(self, fmt, *args):
        return


def build_handler(directory, state, service):
    def _handler(*args, **kwargs):
        return AppHandler(*args, directory=directory, state=state, service=service, **kwargs)

    return _handler


def main(args):
    logging.basicConfig(
        format="%(levelname)s: %(message)s",
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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Run SUMO and serve a live HTML visualizer"
    )

    parser.add_argument(
        "--sumo-cfg",
        default="networks/highway/highway.sumocfg",
        help="Path to .sumocfg",
    )

    parser.add_argument(
        "--sumo-gui",
        action="store_true",
        default=False,
        help="Run SUMO with GUI too",
    )

    parser.add_argument(
        "--step-length",
        type=float,
        default=0.1,
        help="Simulation step length [s]",
    )

    parser.add_argument(
        "--sim-time",
        type=float,
        default=600.0,
        help="Total simulation time [s]",
    )

    parser.add_argument(
        "--client-order",
        type=int,
        default=1,
        help="TraCI client order",
    )

    parser.add_argument(
        "--view-schema",
        default="real world",
        help="SUMO-GUI color scheme",
    )

    parser.add_argument(
        "--lateral-resolution",
        type=float,
        default=0.4,
        help="SUMO lateral resolution for sublane motion [m]",
    )

    parser.add_argument(
        "--http-host",
        default="127.0.0.1",
        help="HTTP bind host",
    )

    parser.add_argument(
        "--http-port",
        type=int,
        default=8000,
        help="HTTP bind port",
    )

    parser.add_argument(
        "--realtime",
        action="store_true",
        default=True,
        help="Throttle to wall-clock time",
    )

    parser.add_argument(
        "--debug",
        action="store_true",
    )

    args = parser.parse_args()
    main(args)