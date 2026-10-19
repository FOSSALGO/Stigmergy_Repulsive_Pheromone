# Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy

**JavaFX-based multi-drone exploration simulator — experimental implementation, playback v2**

A research-oriented grid simulation for studying **decentralized multi-drone coverage**, **frontier-based exploration**, and **repulsive digital pheromone stigmergy**. Each drone independently maintains its own occupancy map, observes the shared *environmental* pheromone field only within its local sensor range, chooses a reachable frontier using a pheromone-aware utility function, and travels toward that frontier using A*.

> **Research status:** This repository contains an experimental implementation for studying the method described by the paper titled above. Parameter values are preliminary; this README does not claim that the proposed method has been empirically shown to outperform its baselines. The program was previously checked with Java 21 and graphics stubs for core logic, but full JavaFX GUI verification must be performed in the user's environment.

## 1. Research idea

The system combines three components:

1. **Decentralized swarm coverage.** Every drone maintains an independent local occupancy map and selects its own exploration target. No centralized frontier allocation or map sharing is used for agent decisions.
2. **Frontier-based exploration.** A frontier is a *known free* grid cell adjacent to at least one *unknown* cell. Frontiers are tracked through an incremental **FTFD-style frontier-tracing update**. Implementation equivalence to a specific published FTFD variant should be validated against that method's reference before claiming exact reproduction.
3. **Repulsive pheromone stigmergy.** A drone deposits pheromone only when it **physically enters** a grid cell. Traces evaporate over time. Drones locally observe the traces to discourage targeting frontiers near already visited areas. Pheromone affects **frontier selection**, not A* edge costs.

### Frontier definition

For drone *i* at time *t*:

```text
F_i(t) = { f in KnownFree_i(t) | N8(f) contains at least one Unknown_i(t) }
```

`N8` is the eight-connected neighbourhood. Candidate targets must be reachable through the drone's own known-free cells.

### Frontier selection

```text
S_i(f) = IG_norm_i(f) - Distance_norm_i(f) - lambda * Pheromone_norm_i(f)
f*_i   = argmax over reachable frontiers of S_i(f)
```

- **Information gain:** predicted number of unknown cells observable from a candidate frontier using local map information and the radial sensing geometry.
- **Travel distance:** A* path cost through known-free cells, normalized by a fixed distance reference.
- **Pheromone penalty:** locally estimated average pheromone in the neighbourhood of the candidate frontier, normalized using a fixed pheromone scale.

### Pheromone update

```text
P(c, t+1) = min(Pmax, (1 - rho) * P(c, t) + Q * visitsEntering(c, t))
```

Only **cell entries** generate deposits; merely sensing an area does not. No pheromone diffusion is used in the primary configuration. A pheromone value that has **never been observed** is assigned the **A1.1 neutral prior** (zero *decision penalty*), which is not a claim that the actual environmental concentration is zero. Previously observed values are estimated using evaporation between observations.

### Motion and collision rules

- **Eight-connected movement:** horizontal/vertical step cost `1`, diagonal cost `sqrt(2)`.
- **No corner cutting:** diagonal movement through blocked corners is prohibited.
- **A* planning:** movement cost depends on distance; no pheromone term is added to its edges.
- **Same destination conflict:** drone with the **smallest ID** receives priority.
- **Other collision checks:** prevent entry into an occupied stationary cell, head-on swaps, and intersecting simultaneous movement segments.
- **Synchronous timesteps:** local observations and decisions precede a single conflict-resolution and pheromone-update phase.

> Movement arbitration belongs to the simulator's physical execution layer. It is **not** centralized target/frontier assignment. Translating this arbitration to independent physical robots would require an appropriate distributed safety mechanism.

## 2. Source code structure

The package is `swarmdrone.decentralizedswarmcoverage`.

```text
src/main/java/swarmdrone/decentralizedswarmcoverage/
├── Launcher.java                  Application launcher
├── MainView.java                  JavaFX window, buttons and event handlers
├── CanvasView.java                Grid editor, drawing, replay and .sc file I/O
├── Drone.java                     Local sensing, local map, FTFD, utility and A*
├── DecentralizedExploration.java  Simulation clock, pheromone, collisions, metrics
├── Point.java                     Grid row/column value object
├── State.java                     Editor interaction modes
├── CSS.java                       JavaFX styles
└── SVGIcon.java                   Toolbar SVG icon definitions
```

**Responsibilities:**

| File | Main responsibility |
| --- | --- |
| `Drone.java` | Autonomous drone logic: 360° radial sensing, private occupancy and pheromone memory, incremental frontiers, expected information gain, A*, target selection, recorded trajectory |
| `DecentralizedExploration.java` | Shared simulation physics, environmental pheromone, visit tracking, ID-priority movement arbitration, aggregate observation timestamps used **only by replay**, and performance metrics |
| `CanvasView.java` | Rendering the world and drones, editing walls and drone positions, playback clock, dark-to-light exploration visualization, `.sc` Open/Save |
| `MainView.java` | JavaFX toolbar, playback controls, Open/Save dialogs, status and log components |

`DecentralizedExploration` can access ground truth for simulating sensors and evaluating results. **Drone decisions rely on locally observed data**, rather than on a shared decision map.

## 3. Requirements and running the application

- A working **Java 21** JDK installation (the core implementation has been checked using Java 21).
- **JavaFX** dependencies compatible with the existing application.
- **IntelliJ IDEA** or another Java IDE; the original project uses a Maven-oriented setup.

**Important:** The distributed source package contains the nine Java classes, not a complete standalone Maven configuration. Use it with your existing working project's `pom.xml`, JavaFX modules/dependencies, resources (including the application icon), and run configuration. Do not replace those files unnecessarily.

1. Back up the currently working project.
2. Place the Java files in `src/main/java/swarmdrone/decentralizedswarmcoverage/`, preserving the package name.
3. Reload the Maven project and confirm JavaFX dependencies are resolved.
4. Run `Launcher.main()` through the existing JavaFX run configuration, or use the existing Maven execution goal if configured in your `pom.xml`.
5. Design a small environment with obstacles and two or three drones, then click **Run**.

**Recommended first test:** Create a simple environment, place two drones at different free cells, run the simulation, pause and resume playback, replay from the beginning, save the environment to `.sc`, then reopen it.

## 4. Application controls

| Control | Behaviour |
| --- | --- |
| **New Environment** | Resets the grid, drones, and existing replay |
| **Open** | Loads a `.sc` environment and drone starting positions |
| **Save** | Saves the environment and drone starting positions to `.sc` |
| **Transform** | Default interaction mode; drag an initial drone position and use middle-mouse panning |
| **Add Drone** | Places a drone at the clicked grid cell |
| **Remove Drone** | Removes a drone at its initial position |
| **Wall** | Toggles individual obstacle cells |
| **Draw Line / Remove Line** | Draws or removes obstacle segments |
| **Run** | Precomputes a simulation and begins the synchronized replay from timestep 0 |
| **Play / Pause** | Pauses or resumes replay; when replay has ended, starting again returns to the beginning |
| **Replay** | Restarts the previously computed trajectory and exploration illumination from the beginning |

The canvas supports mouse-wheel zoom and middle-button panning. The algorithm operates on grid/world coordinates, so changing the view does not change exploration decisions.

### Replay model

**Run performs the simulation first.** The result, including trajectories and the first-observed timestamp of each cell, is then played back as an animation. **Replay does not rerun the algorithm** or recalculate pheromone decisions. All agents share the same replay timestep; waiting steps are represented explicitly to prevent animation desynchronization.

The playback interval is configured in `CanvasView.java`:

```java
private static final long PLAYBACK_INTERVAL_NS = 95_000_000L;
```

This represents approximately **10.5 simulation timesteps per second**. Modifying the interval changes animation speed, **not** the underlying simulation result. A new Run is needed after changing the environment or starting positions.

## 5. Visual explanation: unexplored vs explored

The replay shows the gradual transition of cells as drones observe and visit the environment:

| Appearance | Meaning |
| --- | --- |
| **Dark gray-blue** | Reachable free area that has **not yet been observed** by any drone by the replay timestamp |
| **Light grid** | Area that has become **observed / known** via local radial sensing |
| **Pale green** | A cell **physically visited** by at least one drone |
| **Turquoise obstacle** | Wall / occupied ground-truth cell in the environment renderer |
| **Orange radial region** | Existing drone sensor visibility visualization, bounded by obstacles |
| **Red path** | Previously executed drone trajectory revealed during playback |

The dark-to-light overlay uses the **union of individual local observations for visualization only**. It is not transmitted as a shared occupancy map to drones. The distinction between *observed* and *visited* matters: sensing updates local occupancy knowledge, while **physical visits** drive pheromone deposits and the principal coverage metrics.

## 6. Environment files (`.sc`)

The **Open** and **Save** functions work with a plain-text `.sc` format containing grid size, occupancy cells and initial drone positions. `0` denotes a free environment cell and `-1` denotes an obstacle in the **saved ground-truth grid**; a drone's own unknown/known state is **not** stored in this format.

A small schematic example:

```text
ENVIRONMENT----------------------------------
NUM ROWS:3
NUM COLS:4
0:0:0:0:
0:-1:0:0:
0:0:0:0:
DRONES---------------------------------------
NUM DRONES:2
0:0
2:3
```

Files produced by the earlier application, including the `ENVIRONMENT...` and `DRONES...` headers, are supported by the v2 parser. The loader validates grid dimensions, occupancy values, drone positions and duplicate starting positions before replacing the currently loaded environment.

**Save does not store** pheromone grids, local maps, computed paths, animation position, final metrics or simulation checkpoints. To reproduce a saved environment's simulation, load it and press **Run** again.

## 7. Preliminary experimental parameters

The following values are found in the current **`DecentralizedExploration.java`** implementation. They are **starting values, not validated optima**.

| Parameter | Current value | Meaning |
| --- | ---: | --- |
| `Q` | `1.0` | Pheromone deposit per entered cell |
| `RHO` | `0.05` | Evaporation rate per timestep |
| `PHEROMONE_MAX` | `10.0` | Pheromone saturation and normalization scale |
| `LAMBDA` | `0.5` | Relative frontier pheromone penalty |
| `DISTANCE_REFERENCE` | `50.0` | Fixed A* travel-distance normalization reference |
| `MAX_STEPS` | `1200` | Maximum simulation timesteps |
| `sensorFoV` | `4` cells | Default radial sensor range parameter in `CanvasView` |
| `R_n` | `1` cell | Default frontier pheromone neighbourhood radius in `Drone` |
| Ray directions | `360` | Angular samples for radial occupancy sensing |
| Diffusion | `0` | No diffusion in the primary formulation |

The sensor's effective radial tracing extent in `Drone.observe()` is approximately `sensorFoV + 0.5` cell units, following the previous visualization convention. The occupancy ray-marching step is `0.25` cells. These discretization choices are part of the implementation and should be controlled during experimental comparisons.

**Parameter selection:** Evaluate the sensitivity of `LAMBDA`, evaporation, pheromone saturation, sensor range and normalization before publishing performance conclusions. Avoid choosing parameters using the same test environments/seeds later used for final benchmark claims.

## 8. Performance metrics

The simulator tracks physical visits and exposes these metrics via `DecentralizedExploration`:

| Metric | Definition | Interpretation |
| --- | --- | --- |
| **Coverage ratio** | `uniqueVisitedFreeCells / reachableFreeCells × 100%` | Fraction of reachable free space physically visited by the swarm |
| **Revisit rate** | `(totalVisits - uniqueVisitedFreeCells) / totalVisits × 100%` | Relative frequency of repeat physical visits |
| **Overlap rate** | `cellsVisitedByTwoOrMoreDistinctDrones / uniqueVisitedFreeCells × 100%` | Share of visited cells jointly traversed by multiple agents |

A globally collected visited-cell map and first-observation timestamps are allowed for **evaluation and rendering**, never as target-selection inputs to individual drones.

### Recommended scientific evaluation

Compare at least these three methods under identical environments, sensor ranges, start positions, movement rules and time horizons:

1. **Nearest Frontier** — distance-based baseline.
2. **Frontier Utility without Pheromone** — information gain minus travel cost.
3. **Proposed Pheromone-Aware Frontier Utility** — information gain minus travel cost minus local repulsive pheromone penalty.

**Note:** The current code implements the *proposed method*; it does **not** provide an automated experimental switch for all three comparison methods. Additional baseline implementations and multi-seed experimental runs are needed to substantiate comparative claims.

## 9. Known limitations and development notes

- **Precomputed Run:** Running the entire simulation is synchronous; with a large grid or many agents the JavaFX window may temporarily become unresponsive. Replay itself is time-stepped, but computation is not yet moved to a background worker.
- **FTFD verification:** `Drone.updateFTFD()` maintains frontiers incrementally and traces their boundary. Validation against the specific published FTFD algorithm and against a reference frontier definition is still required before claiming exact FTFD conformity.
- **Rendering is global, decisions are local:** Ground-truth walls and the global explored-area visualization are visible to the human observer, but do not constitute a shared map for drone target selection.
- **Fixed ID priority:** The smallest-ID conflict policy is deterministic and collision-aware, but may cause prolonged waiting or deadlock in congested layouts.
- **No full simulation checkpointing:** `.sc` files save the initial world layout, not a time-resumable execution state.
- **Baseline evaluation:** Nearest Frontier and no-pheromone variants are not yet selectable from the GUI.
- **Parameter calibration:** The numeric defaults, including the fixed distance normalization reference, are exploratory and not optimized.
- **Validation:** Comprehensive GUI, performance, long-duration, and multi-environment scientific verification are still necessary.

## 10. Suggested study order

1. **`Drone.observe()`** — radial sensing and local map updates.
2. **`Drone.updateFTFD()`** — incremental frontier discovery.
3. **`Drone.expectedGain()`**, **`Drone.frontierPheromone()`**, **`Drone.propose()`** — frontier scoring and local target selection.
4. **`Drone.astar()`** — path search on known-free cells.
5. **`DecentralizedExploration.simulate()`** — synchronized environment steps and pheromone evolution.
6. **`DecentralizedExploration.resolveMoves()`** — smallest-ID destination priority and collision rules.
7. **`CanvasView` playback methods** and **`MainView` button handlers** — animation and user interaction.

## 11. Academic context and attribution

**Research title:** *Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy*.

This README documents the present program behaviour and the research design agreed for this project. It is **not** a replacement for the manuscript's complete mathematical definitions, experimental protocols, references or final results. Add the manuscript's definitive author details, paper link/DOI and the references for FTFD, A*, and pheromone-based stigmergy when those bibliographic details have been finalized.

**License:** No software license is specified in this source distribution. Add a `LICENSE` file if public reuse or redistribution is intended.
