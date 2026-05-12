# SUMO Networks

This directory contains runnable SUMO network bundles.

## Active Network

The live server default is:
- `networks/highway/highway.sumocfg`

This is set in `sumo_live_server.py` as the default for `--sumo-cfg`.

## Current Bundles

- `highway/`
  - `highway.sumocfg`
  - `highway.net.xml`
  - `routes_highway.rou.xml`

## Add a New Bundle

For each new network, create one folder under `networks/` and include:
- `<name>.sumocfg`
- `<name>.net.xml`
- `<routes>.rou.xml`

Run it with:

```bash
python3 sumo_live_server.py --sumo-cfg networks/<folder>/<config>.sumocfg
```
