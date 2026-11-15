"""Drone types and their sensor models.

The four types mirror the AERKI payload families. Sensor parameters are modelling assumptions,
chosen to be plausible, not measured; they live in SENSORS so they are easy to change.
"""
from dataclasses import dataclass, field

import numpy as np

SENSORS = {
    # swath: ground width covered per pass (m); p_detect: chance to flag a hotspot inside the swath,
    # as a function of canopy density c; false_rate: false candidates per km flown
    'PA': dict(name='phased-array radar', swath=60.0, speed=12.0,
               p_detect=lambda c, s: 0.55 * s,                       # radar sees through canopy
               false_rate=0.05),
    'EO': dict(name='thermal / multispectral', swath=50.0, speed=12.0,
               p_detect=lambda c, s: 0.95 * s * (1.0 - c) ** 2,      # blocked by canopy
               false_rate=0.10),
    'AQ': dict(name='air-quality sampler', swath=0.0, speed=10.0,
               p_detect=None, false_rate=0.0),                        # confirms via gas, does not search
    'CP': dict(name='edge compute / relay', swath=0.0, speed=10.0,
               p_detect=None, false_rate=0.0),                        # mesh relay and fusion
}
GAS_CONFIRM = 0.05            # relative concentration that counts as smoke at the candidate
UPWIND_CHECK = 60.0           # m: second sample this far upwind to tell a source from drifting smoke
SOURCE_RATIO = 0.5            # the upwind sample must be below this fraction of the candidate's
GAS_NOISE = 0.01              # 1-sigma sensor noise on each sample (relative units)
CONFIRM_RADIUS = 40.0         # m: a confirmed source is attributed to hotspots within this radius


@dataclass
class Drone:
    id: int
    kind: str
    x: float
    y: float
    route: list = field(default_factory=list)   # waypoints still to fly [(x, y), ...]
    task: object = None                         # current confirmation task (Candidate) for AQ drones
    sample: list = field(default_factory=list)  # readings taken for the current task
    flown_m: float = 0.0
    battery_wh: float = 60.0
    landed: bool = False                        # mission flown and back at launch: no sensing
    inbox: list = field(default_factory=list)   # candidates waiting to be shared over the mesh

    @property
    def spec(self):
        return SENSORS[self.kind]

    def step_towards(self, tx, ty, dt):
        dx, dy = tx - self.x, ty - self.y
        d = float(np.hypot(dx, dy))
        step = self.spec['speed'] * dt
        if d <= step:
            self.x, self.y = tx, ty
            self.flown_m += d
            return True
        self.x += dx / d * step
        self.y += dy / d * step
        self.flown_m += step
        return False


@dataclass
class Candidate:
    id: int
    x: float
    y: float
    t: float
    by: int
    hotspot: object = None        # the true hotspot behind it, or None for a false alarm
    assigned_to: int = None
    confirmed: bool = False
    rejected: bool = False
