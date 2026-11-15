"""Range-limited mesh network between drones and the ground station."""
import numpy as np

GCS = -1    # node id of the ground station


def components(nodes, radio_range):
    """Connected components of the mesh. nodes: {id: (x, y)}. Returns {id: component index}."""
    ids = list(nodes)
    comp = {}
    c = 0
    for start in ids:
        if start in comp:
            continue
        stack = [start]
        comp[start] = c
        while stack:
            a = stack.pop()
            ax, ay = nodes[a]
            for b in ids:
                if b not in comp and np.hypot(nodes[b][0] - ax, nodes[b][1] - ay) <= radio_range:
                    comp[b] = c
                    stack.append(b)
        c += 1
    return comp


def hops(nodes, radio_range, src, dst):
    """Minimum hop count between two nodes, or None if they are not connected."""
    if src == dst:
        return 0
    frontier, seen, n = [src], {src}, 0
    while frontier:
        n += 1
        nxt = []
        for a in frontier:
            ax, ay = nodes[a]
            for b, (bx, by) in nodes.items():
                if b not in seen and np.hypot(bx - ax, by - ay) <= radio_range:
                    if b == dst:
                        return n
                    seen.add(b)
                    nxt.append(b)
        frontier = nxt
    return None
