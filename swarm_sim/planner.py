"""Coverage planning and task allocation."""
import numpy as np


def lawnmower(x0, x1, y0, y1, spacing):
    """Boustrophedon waypoints covering the rectangle [x0,x1] x [y0,y1] with north-south lanes."""
    lanes = max(1, int(np.ceil((x1 - x0) / spacing)))
    step = (x1 - x0) / lanes
    pts = []
    for k in range(lanes):
        x = x0 + (k + 0.5) * step
        ys = (y0, y1) if k % 2 == 0 else (y1, y0)
        pts += [(x, ys[0]), (x, ys[1])]
    return pts


def assign_strips(drones, width, height, overlap=0.9):
    """Split the area into vertical strips per sensor type, so every type covers the whole area once.

    Each type's strips are proportional in count to its drones; a hotspot is therefore seen once by
    each sensing modality, which is what makes the mix complementary.
    """
    by_kind = {}
    for d in drones:
        if d.spec['swath'] > 0:
            by_kind.setdefault(d.kind, []).append(d)
    for kind, group in by_kind.items():
        n = len(group)
        for k, d in enumerate(group):
            xa, xb = width * k / n, width * (k + 1) / n
            d.route = lawnmower(xa, xb, 0.0, height, d.spec['swath'] * overlap)


def allocate(candidates, aq_drones, reachable):
    """Greedy nearest-first allocation of unassigned candidates to idle sampler drones.

    reachable(candidate, drone) says whether the candidate is known to that drone through the mesh.
    Returns the (candidate, drone) pairs assigned this step.
    """
    pairs = []
    idle = [d for d in aq_drones if d.task is None]
    open_ = [c for c in candidates if c.assigned_to is None and not (c.confirmed or c.rejected)]
    while idle and open_:
        best = None
        for c in open_:
            for d in idle:
                if not reachable(c, d):
                    continue
                dist = np.hypot(c.x - d.x, c.y - d.y)
                if best is None or dist < best[0]:
                    best = (dist, c, d)
        if best is None:
            break
        _, c, d = best
        c.assigned_to, d.task = d.id, c
        pairs.append((c, d))
        open_.remove(c)
        idle.remove(d)
    return pairs
