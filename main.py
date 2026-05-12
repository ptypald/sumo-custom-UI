#!/usr/bin/env python

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import argparse
import logging
import os
import sys
import time
import random
import math

import lxml.etree as ET

from analytical import *


# ==================================================================================================
# -- SUMO tools path -------------------------------------------------------------------------------
# ==================================================================================================

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib          # noqa: E402
import traci            # noqa: E402

try:
    from traci.exceptions import TraCIException     # noqa: E402
except Exception:  # fallback (very old SUMO)
    try:
        from traci import TraCIException  # type: ignore  # noqa: E402
    except Exception:
        class TraCIException(Exception):
            pass

# ==================================================================================================
# -- helpers ---------------------------------------------------------------------------------------
# ==================================================================================================

def write_sumocfg_xml(cfg_file, net_file, vtypes_file, viewsettings_file, additional_traci_clients=0):
    """Writes a minimal SUMO configuration xml file."""
    root = ET.Element('configuration')
    input_tag = ET.SubElement(root, 'input')
    ET.SubElement(input_tag, 'net-file', {'value': net_file})
    ET.SubElement(input_tag, 'route-files', {'value': vtypes_file})
    gui_tag = ET.SubElement(root, 'gui_only')
    ET.SubElement(gui_tag, 'gui-settings-file', {'value': viewsettings_file})
    ET.SubElement(root, 'num-clients', {'value': str(additional_traci_clients + 1)})
    ET.ElementTree(root).write(cfg_file, pretty_print=True, encoding='UTF-8', xml_declaration=True)

def parse_sumocfg(cfg_path):
    """Extract net-file, route-files, gui-settings-file from a .sumocfg."""
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

# ==================================================================================================
# -- SUMO simulation wrapper -----------------------------------------------------------------------
# ==================================================================================================

class SumoRunner:
    """Starts SUMO (or SUMO-GUI) via TraCI and steps the simulation at a fixed step length."""

    def __init__(self, cfg_file, step_length=0.05, sumo_gui=False,
                 time_to_teleport=None, collision_action=None,
                 host=None, port=None, client_order=1):
        self.step_length = float(step_length)
        bin_name = "sumo-gui" if sumo_gui else "sumo"

        cmd = [
            bin_name,
            "-c", cfg_file,
            "--step-length", str(self.step_length),
            # "--start",
            "--quit-on-end", "false",
        ]
        if time_to_teleport is not None:
            cmd += ["--time-to-teleport", str(time_to_teleport)]
        if collision_action is not None:
            cmd += ["--collision.action", str(collision_action)]

        # For remote SUMO (if host/port provided) one could use traci.init, but here we start locally:
        traci.start(cmd, label=str(client_order))
        self.label = str(client_order)

    def tick(self):
        traci.simulationStep()

    def close(self):
        try:
            traci.close(False)
        except Exception:
            pass

# ==================================================================================================
# -- Builder for SUMO runs --------------------------------------------------------------------
# ==================================================================================================

class WorldBuilder:
    """Builds a SUMO-only run using an existing .sumocfg."""

    def __init__(self, args):
        self.args = args
        self.sumo_net = None
        self.runner = None

    def build(self):
        cfg_file = self.args.sumo_cfg
        net_file, _, _ = parse_sumocfg(cfg_file)
        if not os.path.isabs(net_file):
            net_file = os.path.join(os.path.dirname(cfg_file), net_file)
        self.sumo_net = sumolib.net.readNet(net_file)

        self.runner = SumoRunner(
            cfg_file,
            step_length=self.args.step_length,
            sumo_gui=self.args.sumo_gui,
        )
        return self.sumo_net, self.runner

    def cleanup(self):
        pass

# ==================================================================================================
# -- my functions / classes ------------------------------------------------------------------------
# ==================================================================================================
class Ego:
    def __init__(self):
        self.egoID = None
        self.x = None
        self.v = None
        self.a = None

class TLS:
    def __init__(self):
        self.phase = None
        self.lastPhase = None
        self.timeToSwitch = None

def select_ego(new_vehicle_ids, ego_vehicles, penetration=0.1, rng=None):
    if rng is None:
        rng = random
    chosen = []
    for vid in new_vehicle_ids:
        if rng.random() < float(penetration):
            ego_vehicles.add(vid)
            try:
                traci.vehicle.setColor(vid, (255, 0, 0, 255))
                traci.vehicle.setSpeedMode(vid, 0)
            except Exception:
                pass
            chosen.append(vid)
    return chosen

def vehicle_enters():
    return list(traci.simulation.getDepartedIDList())

def vehicle_exits():
    return list(traci.simulation.getArrivedIDList())

TLS_STATE = {}  # tid -> TLS()
def update_tls_vars():
    t = traci.simulation.getTime()
    for tid in traci.trafficlight.getIDList():
        tls = TLS_STATE.setdefault(tid, TLS())

        cur = traci.trafficlight.getPhase(tid)
        tls.lastPhase = tls.phase
        tls.phase = cur

        try:
            next_switch = traci.trafficlight.getNextSwitch(tid)  # absolute sim time
            tls.timeToSwitch = max(0.0, next_switch - t)
        except TraCIException:
            tls.timeToSwitch = None

def time_to_green_for_link(tls_id, link_idx):
    try:
        now = traci.simulation.getTime()
        cur_phase = traci.trafficlight.getPhase(tls_id)
        next_switch = traci.trafficlight.getNextSwitch(tls_id)
        rem_in_cur = max(0.0, next_switch - now)

        # Full signal program (we assume first program is active)
        defs = traci.trafficlight.getCompleteRedYellowGreenDefinition(tls_id)
        if not defs:
            return None
        phases = defs[0].phases  # list of Phase objects with .state (str) and .duration (float)
        P = len(phases)
        if P == 0:
            return None

        # If already green for this link, time-to-green is zero
        try:
            if phases[cur_phase].state[link_idx] in ('g', 'G'):
                return 0.0
        except IndexError:
            return None

        # Start from NEXT phase; accumulate until we see green on this link
        t_acc = rem_in_cur
        i = (cur_phase + 1) % P
        for _ in range(P):  # at most one full cycle
            st = phases[i].state
            if 0 <= link_idx < len(st) and st[link_idx] in ('g', 'G'):
                return t_acc
            t_acc += float(phases[i].duration)
            i = (i + 1) % P
        return None
    except TraCIException:
        return None

def idm_accel (v , gap , dv , v0=13.9 , a=1.0 , b=2.0 , s0=2.0 , T=1.5 , delta=4):
    s_star = s0 + max (0.0 , v * T + v * dv /(2*math.sqrt(a * b)))
    return a * (1.0 - ( v / v0 )** delta - (s_star / max(gap, 1e-3))**2)

def ego_idm(veh_id, dt):
    try:
        v = traci.vehicle.getSpeed(veh_id)

        # Find leader within 200m
        best_gap = 1e6
        best_dv  = 0.0
        leader_info = traci.vehicle.getLeader(veh_id, 200.0)
        if leader_info is not None:
            leader_id, gap = leader_info
            v_lead = traci.vehicle.getSpeed(leader_id)
            best_gap = max(gap, 1e-3)
            best_dv = v - v_lead

        tls_list = traci.vehicle.getNextTLS(veh_id)
        if tls_list:
            _, _, dist, state = tls_list[0]
            # Treat red/yellow as stop
            blocking = {'r', 'R', 'y', 'Y'}
            if state in blocking and dist <= 200:
                s0 = 2.0
                gap = max(dist - s0, 1e-3)
                if gap < best_gap:
                    best_gap = gap
                    best_dv = v - 0.0

        

        a_cmd = idm_accel(v, best_gap, best_dv)

        # Calculate speed
        new_speed = max(0.0, v + a_cmd * dt)

        # Clip to current lane speed limit
        try:
            lane_id = traci.vehicle.getLaneID(veh_id)
            v_limit = traci.lane.getMaxSpeed(lane_id)
            new_speed = min(new_speed, v_limit)
        except Exception:
            pass

        return new_speed

    except TraCIException:
        pass

def ego_oc(veh_id, dt, s0=5.0):
    try:
        v0 = traci.vehicle.getSpeed(veh_id)
        ve = 15.0
        x0 = 0.0
        N  = 100
        eps = 1e-3

        # --- Next signal info ---
        tls_list = traci.vehicle.getNextTLS(veh_id)
        has_tls = bool(tls_list)
        if has_tls:
            tls_id, link_idx, dist_to_line, state = tls_list[0]
            dist_to_line = max(dist_to_line, 0.0)
        else:
            dist_to_line, state, tls_id = 0.0, 'g', None

        # Final position for both cases
        xe = (dist_to_line + 50.0) if has_tls else x0 + 50.0

        # Constrained trigger & time to green
        is_blocking = state in {'r', 'R', 'y', 'Y'}
        tr = None
        if has_tls and is_blocking:
            try:
                # now = traci.simulation.getTime()
                # tr = traci.trafficlight.getNextSwitch(tls_id) - now
                tr = time_to_green_for_link(tls_id, link_idx)

            except Exception:
                tr = None

        # Intermediate position used only in constrained case
        xr = max(dist_to_line - s0, 0.0)

        if has_tls and is_blocking and tr is not None:
            # --- Constrained case ---
            tr = max(tr, eps)
            v_trap = max((2.0*(xr - x0) / max(tr,1e-9)) - v0, 0.0)
            T_cp   = tr + 2.0*(xe - xr) / max(v_trap + ve, 1e-9)
            T   = max(tr + eps, T_cp, dt + eps)

            oc = ConstrainedOC(x0=x0, v0=v0, xe=xe, ve=ve, T=T, tr=tr, xr=xr, N=N)
            t, x, v_traj, u = oc.solve()
        else:
            # --- Unconstrained case ---
            T = 2.0 * (xe - x0) / max(v0 + ve, 1e-9)
            T = max(T, dt + eps)       # ensure timeline covers dt
            
            oc = UnconstrainedOC(Xf=xe, Vf=ve, T=T, x0=x0, v0=v0, N=N)
            t, x, v_traj, u = oc.solve()

        idx = int(np.clip(np.searchsorted(t, dt, side="left"), 0, len(v_traj) - 1))
        v_next = float(max(0.0, v_traj[idx]))
        return min(v_next, ve)

    except TraCIException:
        return 0.0

# ==================================================================================================
# -- main ------------------------------------------------------------------------------------------
# ==================================================================================================

def main(args):
    builder = WorldBuilder(args)
    sumo_net, runner = builder.build()

    # Optionally set SUMO-GUI color scheme
    if args.sumo_gui and args.view_schema:
        try:
            traci.gui.setSchema(traci.gui.DEFAULT_VIEW, args.view_schema)
            logging.info(f'Set SUMO GUI schema to: {args.view_schema}')
        except Exception as e:
            logging.warning(f'Could not set SUMO GUI schema "{args.view_schema}": {e}')

    ego_vehicles = set()

    try:
        # Example main loop: run for 10 minutes of sim-time
        t_end = 600.0
        rng = random.Random(args.random_seed)

        while traci.simulation.getTime() < t_end:
            start = time.time()

            runner.tick()

            # --- ENTERS ---
            entered = vehicle_enters()
            if entered:
                new_egos = select_ego(entered, ego_vehicles,penetration=args.ego_penetration, rng=rng)
                for vid in entered:
                    tag = " (EGO)" if vid in ego_vehicles else ""
                    print(f"[{traci.simulation.getTime():.1f}s] ENTER  {vid}{tag}")
            
            # --- EXITS ---
            exited = vehicle_exits()
            if exited:
                for vid in exited:
                    tag = " (EGO)" if vid in ego_vehicles else ""
                    print(f"[{traci.simulation.getTime():.1f}s] EXIT   {vid}{tag}")
                    ego_vehicles.discard(vid)

            # --- iterate over all active vehicles ---
            veh_ids = set(traci.vehicle.getIDList())
            for vid in (ego_vehicles & veh_ids):

                new_speed_idm = ego_idm(vid, dt=args.step_length)
                new_speed_anal = ego_oc(vid, dt=args.step_length, s0=2.0)
               
                new_speed = max(min(new_speed_idm, new_speed_anal), 0.0)

                # new_speed = max(new_speed_idm, 0.0)
                # new_speed = max(new_speed_anal, 0.0)
                # Apply new speed
                traci.vehicle.setSpeed(vid, new_speed)
            

            

            # throttle to real-time step if faster than step_length
            # elapsed = time.time() - start
            # if elapsed < args.step_length:
            #     time.sleep(args.step_length - elapsed)
    except KeyboardInterrupt:
        logging.info('Cancelled by user.')
    finally:
        runner.close()
        builder.cleanup()

# ==================================================================================================
# -- CLI -------------------------------------------------------------------------------------------
# ==================================================================================================

if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description="SUMO-only runner (TraCI)")

    argparser.add_argument('--sumo-host', default=None,
                           help='IP of the SUMO host (default: None)')
    argparser.add_argument('--sumo-port', default=None, type=int,
                           help='TCP port for SUMO (default: None)')
    
    argparser.add_argument('--sumo-net', type=str, default=None,
                           help='Path to a .net.xml (used to build a temp .sumocfg).')
    argparser.add_argument('--vtypes-rou', type=str, default=None,
                           help='Path to route/vtypes .rou.xml (when no --sumo-cfg).')
    argparser.add_argument('--viewsettings', type=str, default=None,
                           help='Path to viewsettings.xml (for SUMO GUI; when no --sumo-cfg).')

    argparser.add_argument(
        "--sumo-cfg",
        default="networks/highway/highway.sumocfg",
        help="Path to .sumocfg file",
    )
    argparser.add_argument(
        "--sumo-gui",
        action="store_true",
        default=True,
        help="Run SUMO with GUI (default: True)",
    )
    argparser.add_argument(
        "--step-length", type=float, default=0.1, help="Simulation step length [s]"
    )
    argparser.add_argument(
        "--sim-time", type=float, default=600.0, help="Total simulation time [s]"
    )

    argparser.add_argument('--additional-traci-clients', metavar='TRACI_CLIENTS',
                           default=0, type=int,
                           help='additional TraCI clients to wait for (default: 0)')
    argparser.add_argument('--client-order', metavar='TRACI_CLIENT_ORDER',
                           default=1, type=int,
                           help='client order number for TraCI (default: 1)')

    # Misc / GUI look
    argparser.add_argument('--debug', action='store_true', help='enable debug messages')
    argparser.add_argument('--view-schema', default="real world",
                           help='SUMO-GUI color scheme, e.g., "real world", "standard", "night".')

    argparser.add_argument("--ego-penetration", type=float, default=1.0)
    argparser.add_argument("--random-seed", type=int, default=42)

    args = argparser.parse_args()

    logging.basicConfig(format='%(levelname)s: %(message)s',
                        level=logging.DEBUG if args.debug else logging.INFO)

    main(args)
