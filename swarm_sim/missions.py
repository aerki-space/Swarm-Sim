"""Export each drone's survey route as a QGroundControl WPL 110 mission (loadable in ArduPilot / PX4 tools)."""
import os

from .geo import DEFAULT_ORIGIN, to_latlon

MAV_CMD_NAV_TAKEOFF, MAV_CMD_NAV_WAYPOINT, MAV_CMD_NAV_RETURN_TO_LAUNCH = 22, 16, 20
FRAME_GLOBAL_REL_ALT = 3


def wpl(points, alt=60.0, origin=DEFAULT_ORIGIN, home=(0.0, 0.0)):
    lat0, lon0 = to_latlon(*home, origin)
    rows = ['QGC WPL 110',
            f'0\t1\t0\t{MAV_CMD_NAV_WAYPOINT}\t0\t0\t0\t0\t{lat0:.7f}\t{lon0:.7f}\t0\t1']          # home
    seq = 1
    rows.append(f'{seq}\t0\t{FRAME_GLOBAL_REL_ALT}\t{MAV_CMD_NAV_TAKEOFF}\t0\t0\t0\t0\t0\t0\t{alt:.1f}\t1')
    for x, y in points:
        seq += 1
        lat, lon = to_latlon(x, y, origin)
        rows.append(f'{seq}\t0\t{FRAME_GLOBAL_REL_ALT}\t{MAV_CMD_NAV_WAYPOINT}\t0\t0\t0\t0\t{lat:.7f}\t{lon:.7f}\t{alt:.1f}\t1')
    seq += 1
    rows.append(f'{seq}\t0\t{FRAME_GLOBAL_REL_ALT}\t{MAV_CMD_NAV_RETURN_TO_LAUNCH}\t0\t0\t0\t0\t0\t0\t0\t1')
    return '\n'.join(rows) + '\n'


def export(sim, outdir, alt=60.0, origin=DEFAULT_ORIGIN):
    """Write one .waypoints file per drone that has a planned route. Call before running the sim."""
    os.makedirs(outdir, exist_ok=True)
    paths = []
    for d in sim.drones:
        pts = [p for p in d.route if p != sim.home]
        if not pts:
            continue
        path = os.path.join(outdir, f'{d.kind}-{d.id}.waypoints')
        with open(path, 'w') as f:
            f.write(wpl(pts, alt, origin, home=sim.home))
        paths.append(path)
    return paths
