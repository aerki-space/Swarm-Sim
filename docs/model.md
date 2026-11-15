# Model and assumptions

Swarm-Sim is a decision-level model. It tests coordination strategies (who searches where, who confirms what, and
what the mesh allows) quickly enough to run hundreds of missions. It doesn't model flight dynamics, RF propagation or
image processing. Every parameter below is a modelling assumption, kept in code where it's easy to change. None is a
measured value.

## World (`swarm_sim/world.py`)

- **Area:** a 1.2 × 1.2 km forest on a 10 m grid. Canopy density is 0 – 1, from smoothed noise, so there are patches
  of dense cover tens of metres across.
- **Hotspots:** six per forest (smouldering ground fires, for example), placed preferentially under dense canopy. The
  dangerous ones are the hidden ones.
- **Smoke:** each hotspot releases a Gaussian plume that drifts and widens downwind, with a 2 m/s wind from the
  west-south-west by default.

## Drones (`swarm_sim/agents.py`)

| Type | Role | Swath | Detection model | False alarms |
|---|---|---|---|---|
| PA: phased-array radar | search | 60 m | 0.55 × hotspot strength, independent of canopy | 0.05 per km |
| EO: thermal / multispectral | search | 50 m | 0.95 × strength × (1 − canopy)² | 0.10 per km |
| AQ: air sampler | confirm | – | two-point gas check | – |
| CP: edge compute / relay | relay | – | – | – |

**The key assumption:** a camera can't see a fire through closed canopy, but radar can sense the disturbance under
it. That gives radar a lower peak detection rate but makes it insensitive to cover. The heterogeneous advantage
follows directly from this assumption, so it's the first thing the hardware has to prove.

## Coordination (`swarm_sim/planner.py`, `swarm_sim/mesh.py`, `swarm_sim/sim.py`)

1. **Survey:** each search modality covers the whole area once, split into north-south strips per drone (a
   lawnmower pattern at 90 % of the swath). After their strips, search drones return to launch.
2. **Report:** a detection becomes a candidate held by the reporting drone.
3. **Share:** a candidate reaches a sampler only when the reporter and the sampler are connected through the mesh,
   directly or over several hops, with a 500 m radio range. The relay holds station mid-area to bridge the gap.
4. **Confirm:** the nearest idle sampler that knows about the candidate flies to it and measures gas, then measures
   again 60 m upwind. It confirms only if there's smoke at the candidate and less than half as much upwind. This
   separates a source from smoke drifting past. Sensor noise is 0.01 (relative units).

## Results

From `python -m swarm_sim compare` (30 random forests, 9 drones per fleet):

| | Heterogeneous (3 PA, 3 EO, 2 AQ, 1 CP) | Homogeneous (9 EO) |
|---|---|---|
| Hotspots found | 44 % | 11 % |
| Hotspots confirmed | 44 % | 0 % |
| False alarms left unresolved | 0.0 | 5.3 |
| False alarms wrongly confirmed | 0.1 | 0.0 |
| Mission time | 18.9 min | 8.8 min |

**How to read this:**
- The camera fleet finishes faster, because nine drones share the strips, but it misses most fires under canopy.
  Every false alarm it raises then needs a human to go and look.
- The mixed fleet takes longer and still misses about half the hotspots, because each one gets a single radar pass.
  Two radar passes, or a second look at candidate areas, is the obvious next experiment.

## Known limitations

- **Flight:** straight-line point-mass kinematics; no wind effect on flight, no obstacles.
- **Radio:** binary radio range; no packet loss, latency or bandwidth limits.
- **Sensing:** each drone evaluates a hotspot once, the first time it's inside the swath.
- **Battery:** drain is constant and doesn't end missions.

The roadmap in the README tracks each of these.
