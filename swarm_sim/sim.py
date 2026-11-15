"""Time-stepped swarm simulation."""
from dataclasses import dataclass, field

import numpy as np

from . import mesh, planner
from .agents import CONFIRM_RADIUS, GAS_CONFIRM, SOURCE_RATIO, UPWIND_CHECK, GAS_NOISE, Candidate, Drone
from .world import make_forest

FLEETS = {
    # the AERKI mix: two search modalities that complement each other, samplers to confirm, a relay
    'heterogeneous': {'PA': 3, 'EO': 3, 'AQ': 2, 'CP': 1},
    # the conventional baseline: the same number of identical camera drones
    'homogeneous': {'EO': 9},
}


@dataclass
class Result:
    fleet: str
    seed: int
    duration_s: float
    hotspots: int
    found: int
    confirmed: int
    false_candidates: int
    false_rejected: int
    false_confirmed: int
    false_unresolved: int
    time_to_find: list = field(default_factory=list)      # per found hotspot (s)
    time_to_confirm: list = field(default_factory=list)   # per confirmed hotspot (s)
    coverage: float = 0.0                                 # fraction of the area inside a search swath

    @property
    def found_rate(self):
        return self.found / self.hotspots if self.hotspots else 0.0


class Simulation:
    def __init__(self, fleet='heterogeneous', seed=0, radio_range=500.0, dt=1.0, t_max=3600.0, world=None,
                 on_step=None):
        self.rng = np.random.default_rng(seed + 1000)
        self.world = world or make_forest(seed=seed)
        self.fleet_name = fleet
        self.seed = seed
        self.radio_range = radio_range
        self.dt, self.t, self.t_max = dt, 0.0, t_max
        self.on_step = on_step
        self.drones = []
        for kind, n in FLEETS[fleet].items():
            for _ in range(n):
                self.drones.append(Drone(id=len(self.drones), kind=kind, x=20.0 + 15 * len(self.drones), y=20.0))
        planner.assign_strips(self.drones, self.world.width, self.world.height)
        self.home = (20.0, 20.0)
        for d in self.drones:                    # search drones return to launch after their survey (RTL)
            if d.route:
                d.route.append(self.home)
        self.relays = [d for d in self.drones if d.kind == 'CP']
        for k, d in enumerate(self.relays):     # relays hold station in the middle of the area
            d.route = [(self.world.width * (k + 1) / (len(self.relays) + 1), self.world.height / 2)]
        self.samplers = [d for d in self.drones if d.kind == 'AQ']
        self.candidates = []
        self.seen = set()                        # (drone id, hotspot index) pairs already evaluated
        self.covered = np.zeros_like(self.world.canopy, dtype=bool)
        self.tracks = {d.id: [(d.x, d.y)] for d in self.drones}
        self._comp = {}

    # --- mesh -----------------------------------------------------------------
    def _update_mesh(self):
        nodes = {d.id: (d.x, d.y) for d in self.drones}
        nodes[mesh.GCS] = (0.0, 0.0)
        self._comp = mesh.components(nodes, self.radio_range)

    def connected(self, a, b):
        return self._comp.get(a) is not None and self._comp.get(a) == self._comp.get(b)

    # --- sensing --------------------------------------------------------------
    def _sense(self, d):
        spec = d.spec
        if spec['swath'] <= 0:
            return
        half = spec['swath'] / 2
        w = self.world
        i0, i1 = int(max(0, (d.y - half) // w.cell)), int(min(w.canopy.shape[0] - 1, (d.y + half) // w.cell))
        j0, j1 = int(max(0, (d.x - half) // w.cell)), int(min(w.canopy.shape[1] - 1, (d.x + half) // w.cell))
        self.covered[i0:i1 + 1, j0:j1 + 1] = True
        for k, h in enumerate(w.hotspots):
            if (d.id, k) in self.seen or np.hypot(h.x - d.x, h.y - d.y) > half:
                continue
            self.seen.add((d.id, k))
            if self.rng.random() < spec['p_detect'](w.canopy_at(h.x, h.y), h.strength):
                self._report(d, h.x + self.rng.normal(0, 5), h.y + self.rng.normal(0, 5), h)
        step_km = spec['speed'] * self.dt / 1000.0
        if self.rng.random() < spec['false_rate'] * step_km:
            self._report(d, d.x + self.rng.normal(0, half / 2), d.y + self.rng.normal(0, half / 2), None)

    def _report(self, d, x, y, hotspot):
        c = Candidate(id=len(self.candidates), x=x, y=y, t=self.t, by=d.id, hotspot=hotspot)
        self.candidates.append(c)
        if hotspot is not None and hotspot.found_at is None:
            hotspot.found_at = self.t

    def _reachable(self, c, sampler):
        return self.connected(c.by, sampler.id)   # the reporting drone must be in mesh contact with the sampler

    # --- confirmation ---------------------------------------------------------
    def _sample(self, d):
        """Two-point check: smoke at the candidate, and much less smoke just upwind of it."""
        c, w = d.task, self.world
        speed = max(1e-3, float(np.hypot(*w.wind)))
        ux, uy = w.wind[0] / speed, w.wind[1] / speed
        target = (c.x, c.y) if not d.sample else (c.x - ux * UPWIND_CHECK, c.y - uy * UPWIND_CHECK)
        if not d.step_towards(*target, self.dt):
            return
        d.sample.append(w.gas_at(*target) + self.rng.normal(0, GAS_NOISE))
        if len(d.sample) == 1 and d.sample[0] >= GAS_CONFIRM:
            return                                   # smoke here: go and check upwind before deciding
        at, upwind = d.sample[0], (d.sample[1] if len(d.sample) > 1 else None)
        if upwind is not None and upwind < SOURCE_RATIO * at:
            c.confirmed = True
            for h in w.hotspots:
                if np.hypot(h.x - c.x, h.y - c.y) <= CONFIRM_RADIUS and h.confirmed_at is None:
                    h.confirmed_at = self.t
        else:
            c.rejected = True
        d.task, d.sample = None, []

    # --- main loop ------------------------------------------------------------
    def step(self):
        self._update_mesh()
        planner.allocate(self.candidates, self.samplers, self._reachable)
        for d in self.drones:
            if d.landed:
                self.tracks[d.id].append((d.x, d.y))
                continue
            if d.kind == 'AQ':
                if d.task is not None:
                    self._sample(d)
                else:
                    d.step_towards(*self.home, self.dt)   # idle samplers wait at launch, where the searchers return
            elif d.route:
                if d.step_towards(*d.route[0], self.dt):
                    d.route.pop(0)
                    if not d.route and (d.x, d.y) == self.home:
                        d.landed = True
            self._sense(d)
            d.battery_wh -= 0.012 * self.dt          # ~43 W average draw
            self.tracks[d.id].append((d.x, d.y))
        self.t += self.dt
        if self.on_step:
            self.on_step(self)

    def busy(self):
        searching = any(d.route for d in self.drones if d.kind not in ('CP',))
        pending = self.samplers and any(not (c.confirmed or c.rejected) for c in self.candidates)
        return searching or bool(pending)

    def run(self):
        while self.t < self.t_max and self.busy():
            self.step()
        return self.result()

    def result(self):
        hs = self.world.hotspots
        false = [c for c in self.candidates if c.hotspot is None]
        return Result(
            fleet=self.fleet_name, seed=self.seed, duration_s=self.t, hotspots=len(hs),
            found=sum(h.found_at is not None for h in hs),
            confirmed=sum(h.confirmed_at is not None for h in hs),
            false_candidates=len(false),
            false_rejected=sum(c.rejected for c in false),
            false_confirmed=sum(c.confirmed for c in false),
            false_unresolved=sum(not (c.rejected or c.confirmed) for c in false),
            time_to_find=[h.found_at for h in hs if h.found_at is not None],
            time_to_confirm=[h.confirmed_at for h in hs if h.confirmed_at is not None],
            coverage=float(self.covered.mean()),
        )
