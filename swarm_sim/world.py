"""The search area: canopy density, hidden hotspots and the gas plumes they release.

Coordinates are metres in a local east/north frame with the origin at the area's south-west corner.
"""
from dataclasses import dataclass, field

import numpy as np


@dataclass
class Hotspot:
    x: float
    y: float
    strength: float          # 0..1, how far the smouldering has progressed
    found_at: float = None   # sim time of the first candidate detection (s)
    confirmed_at: float = None


@dataclass
class World:
    width: float
    height: float
    canopy: np.ndarray       # canopy density 0..1 on a regular grid
    cell: float              # grid cell size (m)
    hotspots: list = field(default_factory=list)
    wind: tuple = (2.0, 0.5)  # m/s, east/north

    def canopy_at(self, x, y):
        i = int(np.clip(y // self.cell, 0, self.canopy.shape[0] - 1))
        j = int(np.clip(x // self.cell, 0, self.canopy.shape[1] - 1))
        return float(self.canopy[i, j])

    def gas_at(self, x, y):
        """Relative gas concentration at (x, y): Gaussian plumes downwind of every hotspot."""
        wx, wy = self.wind
        speed = max(1e-3, np.hypot(wx, wy))
        ux, uy = wx / speed, wy / speed
        total = 0.0
        for h in self.hotspots:
            dx, dy = x - h.x, y - h.y
            along = dx * ux + dy * uy
            if along <= 0:
                along_eff = 0.0
                across = np.hypot(dx, dy)
            else:
                along_eff = along
                across = abs(-dx * uy + dy * ux)
            sigma = 8.0 + 0.15 * along_eff               # plume widens downwind
            total += h.strength * np.exp(-0.5 * (across / sigma) ** 2) * np.exp(-along_eff / 400.0) \
                * (1.0 if along > 0 else np.exp(-0.5 * (np.hypot(dx, dy) / 10.0) ** 2))
        return total


def make_forest(width=1200.0, height=1200.0, cell=10.0, n_hotspots=6, seed=0):
    """A forest with smooth patches of dense canopy and hotspots mostly hidden under it."""
    rng = np.random.default_rng(seed)
    ny, nx = int(height // cell), int(width // cell)
    noise = rng.normal(size=(ny, nx))
    # smooth the noise with a separable box blur to get canopy patches tens of metres across
    k = 7
    kernel = np.ones(k) / k
    for _ in range(3):
        noise = np.apply_along_axis(lambda r: np.convolve(r, kernel, mode='same'), 1, noise)
        noise = np.apply_along_axis(lambda c: np.convolve(c, kernel, mode='same'), 0, noise)
    noise = (noise - noise.min()) / (noise.max() - noise.min())
    canopy = np.clip(0.35 + 0.65 * noise, 0.0, 1.0)

    world = World(width=width, height=height, canopy=canopy, cell=cell)
    # place hotspots preferentially where the canopy is dense: the hard, realistic case
    flat = canopy.ravel() ** 4
    idx = rng.choice(flat.size, size=n_hotspots, replace=False, p=flat / flat.sum())
    for k_ in idx:
        i, j = divmod(int(k_), nx)
        world.hotspots.append(Hotspot(x=(j + 0.5) * cell, y=(i + 0.5) * cell, strength=float(rng.uniform(0.5, 1.0))))
    return world
