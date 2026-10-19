# V010 — Decentralized Frontier Exploration with Evaporating Agent-Specific Territorial Markers

Research title (unchanged): **Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy**

## What changed from V007

V010 uses the V007 implementation as its baseline. The original **visited-cell repulsive pheromone** (deposit, evaporation, radial local sensing, multi-scale neighbourhood penalty) is unchanged. A **separate agent-specific temporary territorial marker** is introduced as an additional indirect coordination signal. The marker is **not** deposited on all observed cells as visited pheromone; it is a distinct short-range digital reservation cue which is not counted as a physical visit. The marker for agent `i` is stored in an environment-owned channel `T[i][r][c]`. Other drones may read a marker **only for cells visible through their existing radial sensing**, and the owner's own marker is excluded from its penalty. No global target allocation, direct map sharing, or global marker reading is introduced into agent decision making.

Territorial marks are refreshed at a drone's current position and (when the target lies in the currently sensed footprint) at its active frontier target. Marks decay by `0.82` every timestep; an abandoned claim vanishes exponentially. Unlike the original visited-cell pheromone, a target marker can exist on a cell before physical entry: this is a virtual, sensor-local territorial cue rather than a visitation record. The two channels must not be conflated in the article.

The frontier utility becomes:

`S_i(f) = IG_i(f) - D_i(f) - lambda * P_i_multi(f) - beta * T_i_other(f)`

where `T_i_other` is an average of **locally observed, decaying marks deposited by other owners** over a radius-2 neighbourhood. The marker term is a soft penalty, never an obstacle nor an A* path cost. All frontiers remain selectable, so exploration can recover after a marker expires. Original FTFD, independent local occupancy maps, 8-connected A*, conflict priority by smallest agent ID, and completion-aware termination remain as in V007.

**Research disclosure:** The V010 agent-specific territorial signal is an additional coordination mechanism beyond the original visited-cell stigmergy. This version is not a strict implementation of the previous pheromone-only method; it should be reported as a separate ablation/extension and not described as communication-free unless the distinction between environmental indirect cues and direct messages is made clear.

## Installation

Copy the files to your JavaFX Maven project using the existing package:

`src/main/java/swarmdrone/decentralizedswarmcoverage_ijrcs/`

**Changed Java files:** `Drone.java`, `DecentralizedExploration.java`, `CanvasView.java`. `CanvasView` is changed only where it assigns the path color when a drone is added/loaded, and where it renders the existing path in that drone's color. The drone graphics, radial visibility, obstacles, other overlays and all buttons are otherwise unchanged. `MainView.java`, `CSS.java` and all other Java files match V007.

## Path palette

Exactly 30 pre-defined bright, distinct hex colors are stored in `Drone.PATH_COLORS` and assigned via `setPathColorByIndex()` when the drone is **created** through Add Drone or read from a `.sc` file. A drone keeps its path color across simulation/replay. After 30 drones, colors cycle (they are a list of 30, not an unlimited uniqueness guarantee). No separate user interface element is required.

## Run modes and configuration

- **V007 baseline:** `-Dswarm.mode=BASELINE`
- **V010 territorial:** `-Dswarm.mode=TERRITORY` (default)

Optional experiment parameters:

- `-Dswarm.territoryWeight=1.4` (initial candidate, not optimized)
- `-Dswarm.territoryRadius=2`
- `-Dswarm.territoryRetain=0.82` (per-timestep retention; absence of renewal means decay)
- Original V007 ablations continue to work: `swarm.multiscale` and `swarm.eventReview`.

Original pheromone parameters are unchanged. Do **not** interpret the territorial weight as a replacement for the original pheromone's `lambda`.

## Experimental checks

Run `bash tests/run_v010_comparison.sh`. This compiles algorithmic classes against graphics stubs (not a complete JavaFX GUI test) and runs a six-drone, synthetic corridor environment for each mode.

| Metric | V007 baseline | Territorial, beta = 1.4 |
|---|---:|---:|
| Observed reachable free cells | 100% | 100% |
| Completion time (timesteps) | 128 | 129 |
| Revisit rate | 47.32% | 44.44% |
| Inter-drone overlap rate | 46.90% | 45.12% |

The table describes **one deterministic synthetic test**. The territorial result was selected after experimenting with several weights and radii on this very map; it is **not** an independent performance validation or evidence of an optimal parameter choice. Other weights/radii increased overlap. Repeat on your actual `.sc` map, varied starts, 2/4/6/10/30 drones and independent seeds; assess completion time as a co-primary outcome.

## Important limitations

- Even an effective territorial cue cannot prevent multiple drones choosing the same distant target before either detects the other's local marker.
- The environmental marker is an idealized digital trace sensed locally, **not** a physical animal scent model.
- Temporarily marked goals remain eligible, and marks decay; completion still depends on local exploration, not on an artificial allocation by the simulator.
- Narrow necessary corridors can create overlap that should not be penalized as failed coordination.
- Paired smoke tests and algorithm compilation were performed; a full JavaFX GUI runtime verification in your IntelliJ configuration is still needed.
- Open/Save, Run, Play/Pause, Replay, canvas transformations, and sensor visualization inherit their behavior from V007.
