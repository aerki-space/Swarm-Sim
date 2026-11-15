"""Command line:  python -m swarm_sim {run,compare,missions} ...

  run       one simulation; optional plot and live UDP telemetry for the ground station
  compare   the AERKI heterogeneous fleet vs. a homogeneous camera fleet over many random forests
  missions  export each drone's survey as an ArduPilot / PX4 mission file
"""
import argparse
import statistics
import sys

from . import missions, plot
from .sim import FLEETS, Simulation
from .telemetry import UdpPublisher


def cmd_run(a):
    hook = None
    if a.udp:
        host, port = a.udp.rsplit(':', 1)
        hook = UdpPublisher(host, int(port), realtime=a.realtime)
        print(f'streaming telemetry to udp://{host}:{port} at {a.realtime:g}x real time')
    sim = Simulation(a.fleet, seed=a.seed, radio_range=a.radio_range, on_step=hook)
    r = sim.run()
    print(f'fleet {r.fleet}, seed {r.seed}: {r.duration_s:.0f} s, hotspots found {r.found}/{r.hotspots}, '
          f'confirmed {r.confirmed}, false alarms {r.false_candidates} '
          f'(rejected {r.false_rejected}, unresolved {r.false_unresolved}), coverage {r.coverage:.0%}')
    if a.plot:
        plot.save_single(sim, a.plot)
        print('wrote', a.plot)
    return 0


def summarize(results):
    tf = [t for r in results for t in r.time_to_find]
    tc = [t for r in results for t in r.time_to_confirm]
    return {
        'hotspots found': f"{100 * statistics.mean(r.found_rate for r in results):.0f}%",
        'hotspots confirmed': f"{100 * statistics.mean(r.confirmed / r.hotspots for r in results):.0f}%",
        'median time to find': f'{statistics.median(tf) / 60:.1f} min' if tf else '–',
        'median time to confirm': f'{statistics.median(tc) / 60:.1f} min' if tc else '–',
        'false alarms per mission': f'{statistics.mean(r.false_candidates for r in results):.1f}',
        'false alarms left unresolved': f'{statistics.mean(r.false_unresolved for r in results):.1f}',
        'false alarms wrongly confirmed': f'{statistics.mean(r.false_confirmed for r in results):.1f}',
        'mission time': f'{statistics.mean(r.duration_s for r in results) / 60:.1f} min',
    }


def cmd_compare(a):
    fleets = list(FLEETS)
    table = {f: summarize([Simulation(f, seed=s).run() for s in range(a.seeds)]) for f in fleets}
    rows = list(table[fleets[0]])
    print(f'{a.seeds} random forests, {sum(FLEETS[fleets[0]].values())} drones per fleet\n')
    print(f"| | {' | '.join(fleets)} |\n|---|{'---|' * len(fleets)}")
    for row in rows:
        print(f"| {row} | {' | '.join(table[f][row] for f in fleets)} |")
    if a.plot:
        sims = []
        for f in fleets:
            s = Simulation(f, seed=a.plot_seed)
            s.run()
            sims.append(s)
        plot.save_comparison(sims, a.plot, suptitle=f'Same forest (seed {a.plot_seed}), same number of drones')
        print('\nwrote', a.plot)
    return 0


def cmd_missions(a):
    sim = Simulation(a.fleet, seed=a.seed)
    for p in missions.export(sim, a.out, alt=a.alt):
        print('wrote', p)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(prog='python -m swarm_sim', description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run')
    r.add_argument('--fleet', choices=FLEETS, default='heterogeneous')
    r.add_argument('--seed', type=int, default=0)
    r.add_argument('--radio-range', type=float, default=500.0, help='mesh radio range (m)')
    r.add_argument('--plot', help='write a PNG of the run')
    r.add_argument('--udp', help='stream telemetry to HOST:PORT (e.g. 127.0.0.1:14560)')
    r.add_argument('--realtime', type=float, default=10.0, help='sim speed vs. wall clock when streaming')
    r.set_defaults(fn=cmd_run)
    c = sub.add_parser('compare')
    c.add_argument('--seeds', type=int, default=30)
    c.add_argument('--plot', help='write a side-by-side PNG for one forest')
    c.add_argument('--plot-seed', type=int, default=3)
    c.set_defaults(fn=cmd_compare)
    m = sub.add_parser('missions')
    m.add_argument('--fleet', choices=FLEETS, default='heterogeneous')
    m.add_argument('--seed', type=int, default=0)
    m.add_argument('--alt', type=float, default=60.0, help='survey altitude above launch (m)')
    m.add_argument('--out', default='missions')
    m.set_defaults(fn=cmd_missions)
    a = ap.parse_args(argv)
    return a.fn(a)


if __name__ == '__main__':
    sys.exit(main())
