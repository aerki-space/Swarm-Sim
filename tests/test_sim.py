"""python3 -m unittest discover -s tests"""
import json
import os
import socket
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np  # noqa: E402

from swarm_sim import mesh, missions, planner  # noqa: E402
from swarm_sim.geo import to_latlon, to_local  # noqa: E402
from swarm_sim.sim import Simulation  # noqa: E402
from swarm_sim.telemetry import UdpPublisher, drone_message  # noqa: E402
from swarm_sim.world import Hotspot, make_forest  # noqa: E402


class Planner(unittest.TestCase):
    def test_lawnmower_covers_every_column(self):
        pts = planner.lawnmower(0, 300, 0, 100, spacing=50)
        xs = sorted({x for x, _ in pts})
        self.assertEqual(len(xs), 6)
        gaps = np.diff([0] + xs + [300])
        self.assertLessEqual(max(gaps), 50 + 1e-9)        # no strip wider than the swath is left out

    def test_each_sensor_type_covers_the_whole_area(self):
        sim = Simulation('heterogeneous', seed=0)
        sim.run()
        self.assertGreaterEqual(sim.result().coverage, 0.99)


class Mesh(unittest.TestCase):
    def test_multi_hop(self):
        nodes = {0: (0, 0), 1: (400, 0), 2: (800, 0), 3: (2000, 0)}
        self.assertEqual(mesh.hops(nodes, 500, 0, 2), 2)
        self.assertIsNone(mesh.hops(nodes, 500, 0, 3))
        comp = mesh.components(nodes, 500)
        self.assertEqual(comp[0], comp[2])
        self.assertNotEqual(comp[0], comp[3])


class World(unittest.TestCase):
    def test_gas_is_downwind_not_upwind(self):
        w = make_forest(seed=0, n_hotspots=0)
        w.hotspots = [Hotspot(x=600, y=600, strength=1.0)]
        w.wind = (2.0, 0.0)
        self.assertGreater(w.gas_at(700, 600), 10 * w.gas_at(500, 600))


class Simulation_(unittest.TestCase):
    def test_deterministic(self):
        a, b = Simulation(seed=5).run(), Simulation(seed=5).run()
        self.assertEqual(a, b)

    def test_samplers_resolve_every_false_alarm(self):
        for seed in range(5):
            r = Simulation('heterogeneous', seed=seed).run()
            self.assertEqual(r.false_unresolved, 0, f'seed {seed}')

    def test_heterogeneous_finds_more_under_canopy(self):
        het = [Simulation('heterogeneous', seed=s).run().found_rate for s in range(10)]
        hom = [Simulation('homogeneous', seed=s).run().found_rate for s in range(10)]
        self.assertGreater(np.mean(het), np.mean(hom))

    def test_terminates_before_time_limit(self):
        for seed in range(5):
            self.assertLess(Simulation(seed=seed).run().duration_s, 3600)


class Outputs(unittest.TestCase):
    def test_geo_roundtrip(self):
        lat, lon = to_latlon(1234.0, 567.0)
        x, y = to_local(lat, lon)
        self.assertAlmostEqual(x, 1234.0, places=6)
        self.assertAlmostEqual(y, 567.0, places=6)

    def test_mission_file_format(self):
        sim = Simulation(seed=0)
        with tempfile.TemporaryDirectory() as d:
            paths = missions.export(sim, d)
            self.assertEqual(len(paths), 7)               # 3 radar + 3 thermal surveys, 1 relay station
            rows = open(paths[0]).read().splitlines()
        self.assertEqual(rows[0], 'QGC WPL 110')
        self.assertEqual(rows[2].split('\t')[3], '22')    # takeoff
        self.assertEqual(rows[-1].split('\t')[3], '20')   # return to launch
        self.assertTrue(all(len(r.split('\t')) == 12 for r in rows[1:]))

    def test_telemetry_over_udp(self):
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.bind(('127.0.0.1', 0))
        rx.settimeout(2)
        pub = UdpPublisher('127.0.0.1', rx.getsockname()[1], realtime=0)
        sim = Simulation(seed=0, on_step=pub, t_max=3)
        sim.run()
        msgs = []
        try:
            while True:
                msgs.append(json.loads(rx.recv(65535)))
        except socket.timeout:
            pass
        drones = [m for m in msgs if m['type'] == 'drone']
        self.assertGreaterEqual(len({m['id'] for m in drones}), 9)
        m = drone_message(sim, sim.drones[0])
        self.assertEqual(set(m), {'v', 'type', 't', 'id', 'role', 'lat', 'lon', 'alt', 'heading', 'battery', 'state'})


if __name__ == '__main__':
    unittest.main()
