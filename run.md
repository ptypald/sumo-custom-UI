python3 sumo_live_server.py --sumo-cfg networks/highway/highway.sumocfg --ego-controller csac

# optional: control only specific vehicles (example SUMO IDs)
# python3 sumo_live_server.py --sumo-cfg networks/highway/highway.sumocfg --ego-controller csac --ego-ids flow_main.0 flow_main.1
# recommended for CSAC consistency (controller model uses dt=0.2s)
# python3 sumo_live_server.py --sumo-cfg networks/highway/highway.sumocfg --ego-controller csac --step-length 0.2
# optional: force a specific CSAC template (default is --csac-scenario auto)
# python3 sumo_live_server.py --sumo-cfg networks/highway/highway.sumocfg --ego-controller csac --csac-scenario merge_two_main_lanes
# note: with --csac-scenario auto, CSAC lane centers/width are inferred from the active SUMO edge geometry
# optional: tune lateral smoothness (smaller -> finer continuous sublane motion)
# python3 sumo_live_server.py --sumo-cfg networks/highway/highway.sumocfg --ego-controller csac --lateral-resolution 0.2

http://127.0.0.1:8000
