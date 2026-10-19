# Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy

**V006 — Completion-Aware Exploration and Local Congestion Recovery**

An experimental Java/JavaFX grid-based simulator for studying autonomous multi-drone area exploration. Each drone builds its own occupancy map, detects exploration frontiers, independently selects a target using information gain, travel distance, and locally sensed repulsive pheromone, and navigates through known-free cells using A*.

> **Research status:** This is a research prototype, not a validated collision-free physical-drone controller. Parameter values are experimental defaults rather than established optima. V006 improves mission-completion handling and local recovery from blocked moves; it does **not** guarantee minimal overlap or full completion in every environment.

## Contents

- [Research approach](#research-approach)
- [What's new in V006](#whats-new-in-v006)
- [Core mathematical model](#core-mathematical-model)
- [Software architecture](#software-architecture)
- [Requirements and setup](#requirements-and-setup)
- [Using the simulator](#using-the-simulator)
- [Visualizations and metrics](#visualizations-and-metrics)
- [Experimental parameters](#experimental-parameters)
- [Validation and limitations](#validation-and-limitations)

## Research approach

The simulator combines three components:

1. **Decentralized swarm coverage:** Each agent independently maintains a *local occupancy map* and *local pheromone memory*. Agents do not exchange maps, destinations, or planned paths.
2. **Frontier-based exploration:** Each drone incrementally maintains a frontier set, using a frontier-tracing-style update (FTFD), then plans feasible paths to candidate frontiers.
3. **Repulsive pheromone stigmergy:** Visiting a free cell deposits pheromone in the shared simulated environment. Drones sense this field **only within their local radial sensor footprint** and use locally available pheromone as a **penalty in frontier target utility**, not as a cost in A*.

The simulator centrally administers the physical environment, collision arbitration, measurement, and stopping criterion. **It does not assign exploration targets to agents.** In V006 the completion check uses ground-truth reachability and the union of sensed free cells *only at the simulator level*; it must not be described as a decentralized termination protocol.

### Sensing, mapping, and movement

- **Single radial sensor:** 360-degree ray casting with obstacle occlusion. The same sensor updates known-free / known-occupied occupancy cells and locally sensed pheromone. There is no separate `cameraFoV` sensor in the decision algorithm.
- **Local occupancy states:** `UNKNOWN = 0`, `FREE = 1`, `OCCUPIED = -1`.
- **Frontier:** A known-free cell adjacent to at least one unknown cell, with **8-connected** neighbourhoods.
- **Motion:** 8-connected grid; orthogonal step cost `1`, diagonal cost `sqrt(2)`. Diagonal *corner cutting* through blocked or unconfirmed corner cells is prohibited.
- **Path planning:** A* on each drone's known-free local map. Pheromone is excluded from A* edge costs.
- **Movement conflicts:** When multiple agents request the same destination in a timestep, the smaller drone ID has priority. The simulator also prevents unsafe occupancy/swap/crossing conflicts.

## What's new in V006

V006 builds on the V005 target-selection update and focuses on completion and congestion rather than claiming to solve all redundant exploration.

| Behaviour | V005 issue | V006 implementation |
|---|---|---|
| Mission termination | Could continue after the entire reachable free area had been sensed | Stops when all **reachable free cells have been observed by at least one drone** |
| Local traffic congestion | A rejected move could be repeatedly proposed | After **3 consecutive denied moves**, temporarily avoids the rejected next cell for **8 timesteps** and independently replans |
| Unproductive frontiers | Some candidates had zero predicted gain | Skips candidates with estimated information gain of zero |
| Diagnostics | Completion could be confused with physically visited coverage | Distinguishes `observed ratio` from `visited coverage` and reports a termination reason |

V005 behaviour retained in V006:

- Periodic frontier-target reassessment every **8 timesteps** (`RECHECK_INTERVAL`).
- Target-switch hysteresis margin **0.08** (`SWITCH_MARGIN`) to limit oscillation.
- Pheromone-aware utility with a separate, fixed normalization reference.
- Local frontier memory, incremental FTFD-style updates, A*, and ID-priority collision handling.
- Existing JavaFX interface, file operations, and synchronized exploration replay.

## Core mathematical model

### Frontier detection

For drone `i`, let `Free_i(t)` and `Unknown_i(t)` denote cells in its **own** local map:

```text
F_i(t) = { f in Free_i(t) : N8(f) intersects Unknown_i(t) }
```

Frontiers are maintained incrementally after local sensing. The current implementation follows an incremental frontier-tracing approach; its exact correspondence with a particular published FTFD variant still requires reference-level validation.

### Frontier utility

Each reachable frontier receives a utility score:

```text
S_i(f,t) = IG_hat_i(f,t) - D_hat_i(f,t) - lambda * P_hat_i(f,t)
```

Where:

- `IG_hat` is normalized **expected information gain**, estimated as previously unknown cells likely to be sensed from the candidate frontier.
- `D_hat` is normalized **A* travel distance** on the locally known traversable map.
- `P_hat` is normalized **locally observed neighbourhood pheromone** near that frontier.
- `lambda` controls the strength of the repulsive pheromone penalty.

The drone selects the highest-scoring reachable candidate. The simulation does not perform centralized frontier allocation.

### Environmental pheromone

Pheromone is deposited **only when a drone physically enters a cell**, not when the cell is merely sensed:

```text
P(c,t+1) = min(Pmax, (1-rho) * P(c,t) + Q * visits_entering(c,t))
```

The main algorithm uses evaporation and deposit, **without diffusion**. Local drone memory keeps observed values and timestamps. Never-observed pheromone uses the **A1.1 neutral-prior** operational value `0` without claiming that the real environmental pheromone must be zero.

For utility normalization, **V006 uses a pheromone reference distinct from the environmental maximum**:

```text
P_hat = min(1, neighbourhood_pheromone / PHEROMONE_REFERENCE)
```

This is a V005/V006 experimental choice and should be evaluated through sensitivity tests, not portrayed as a theoretically optimal setting.

### Simulation schedule

```text
Initialize environment, agent-local maps, pheromone, and starting visits
For each simulation timestep:
    All drones sense from the same current environmental snapshot
    Simulator checks observed-area completion
    Each drone independently updates its frontier set and proposes a move
    Simulator resolves simultaneous movement conflicts
    Approved drone movements are applied
    Visited cells receive deposit; pheromone evaporates once per timestep
    Per-timestep positions and observation times are recorded for replay
Stop on observed completion, stall limit, or maximum horizon
```

## Software architecture

Java package: `swarmdrone.decentralizedswarmcoverage`

```text
V006/
├── src/main/java/swarmdrone/decentralizedswarmcoverage/
│   ├── Drone.java                      # Local sensing, FTFD-style frontiers, utility, A*
│   ├── DecentralizedExploration.java   # Simulation loop, pheromone, conflicts, metrics
│   ├── CanvasView.java                 # Grid editor, visualization, playback, .sc files
│   ├── MainView.java                   # JavaFX window, toolbar, controls
│   ├── Launcher.java                   # Application entry point
│   ├── Point.java                      # Grid coordinate representation
│   ├── State.java                      # Editor interaction modes
│   ├── SVGIcon.java                    # Toolbar icons
│   └── CSS.java                        # UI styles
├── SixDroneSmokeTest.java             # Standalone algorithm-level scenario
└── OverlapBenchmark.java              # Additional algorithm-level scenario
```

The distributed V006 source archive **does not include a `pom.xml` or all JavaFX resources**. Use the Maven configuration and resources from the working parent project/V005, retaining its JavaFX dependencies and application icon. The test source files are supplemental harnesses, not GUI entry points.

## Requirements and setup

- **JDK 21** (the version used for algorithm-level checks).
- IntelliJ IDEA or another Java IDE.
- JavaFX libraries and the **existing project Maven configuration**.
- Application resources from the working JavaFX project (including its icon where applicable).

To integrate the V006 algorithm update into a working V005 project:

1. Back up the working V005 project.
2. Copy `Drone.java` and `DecentralizedExploration.java` from V006 into the existing `swarmdrone.decentralizedswarmcoverage` package.
3. Keep the other Java files, project `pom.xml`, and resources from your working application.
4. Build and run the application's `Launcher` / `MainView` using your existing JavaFX run configuration.
5. Reopen your saved `.sc` environment and compare the same drone starting positions.

The project package path is important: keep the `package swarmdrone.decentralizedswarmcoverage;` declaration consistent with your source directories.

## Using the simulator

| Control | Behaviour |
|---|---|
| **New** | Reset the editing environment |
| **Open** | Load an environment and initial drone locations from a `.sc` file |
| **Save** | Save the environment grid and initial drone locations to `.sc` |
| **Add / Remove Drone** | Edit initial drone positions |
| **Wall / Draw Line / Remove Line** | Modify obstacles |
| **Transform, zoom, pan** | Navigate and edit the grid view |
| **Run** | Compute the simulation first, then start synchronized playback |
| **Play/Pause** | Pause or resume the *recorded playback* |
| **Replay** | Restart visualization from timestep 0 without recomputing the algorithm |

**Important:** `Run` performs the algorithmic computation before the animation. Playback is a visualization of recorded trajectories and sensing timestamps; pressing Replay does **not** rerun exploration or change its decisions. The playback timer is approximately **95 ms per timestep** (~10.5 frames of simulated movement per second), independent of the algorithmic timestep.

### Exploration visualization

- **Dark cells:** Reachable free areas not yet observed at the displayed timestep.
- **Light cells:** Areas already observed by at least one drone.
- **Green-tinted cells:** Physically visited cells in the recorded movement trajectories.
- **Red lines:** Recorded movement trajectories, including repeated traversals.
- **Orange radial footprint:** Rendered sensing range around each drone.
- **Turquoise walls:** Obstacles.

The dark-to-light layer is a **visual union of observations**. It is not a merged map shared with drones, and it is not equivalent to physically visited coverage.

## Experimental parameters

The current algorithm classes specify these **initial settings**, which are subject to calibration:

| Parameter | V006 default | Location / meaning |
|---|---:|---|
| Number of sensor rays | 360 | `Drone.java`: full radial sensing |
| Example `sensorFoV` | 4 cells | `CanvasView.java`: editor-configured sensor range |
| `Q` | `1.0` | Deposit per physical cell entry |
| `RHO` | `0.05` | Environmental evaporation per timestep |
| `PHEROMONE_MAX` | `10.0` | Saturation cap of actual field |
| `PHEROMONE_REFERENCE` | `2.0` | Fixed utility-normalization reference |
| `LAMBDA` | `1.0` | Repulsive pheromone utility weight |
| `DISTANCE_REFERENCE` | `50.0` | Fixed travel-distance scale |
| `MAX_STEPS` | `1200` | Upper bound on simulation duration |
| `RECHECK_INTERVAL` | `8` | Target re-evaluation interval |
| `SWITCH_MARGIN` | `0.08` | Minimum utility improvement to switch targets |
| Collision backoff | 3 rejected moves | Local congestion trigger |
| Temporary avoided cell | 8 timesteps | Duration of local avoidance |
| Stall threshold | 30 timesteps | Stop after prolonged lack of movement |

Parameter selection, including `lambda`, pheromone normalization, distance scale, and termination horizon, should be disclosed when reporting results.

## Visualizations and metrics

The following metrics must **not** be conflated:

| Metric | Definition | Interpretation |
|---|---|---|
| **Observed-area ratio** | Unique reachable free cells sensed by ≥1 drone / all reachable free cells | **Determines sensing completion in V006** |
| **Visited coverage ratio** | Unique reachable free cells physically entered / all reachable free cells | Research coverage metric based on actual visits |
| **Revisit rate** | `(total visits - unique visited cells) / total visits` | Fraction of visits that repeat already visited cells |
| **Overlap rate** | Cells visited by ≥2 distinct drones / unique visited cells | Spatial redundancy among agents |

All ratios are multiplied by 100 when displayed as percentages. Starting positions count as visits. Observed 100% **does not imply** physically visited coverage of 100%.

### Termination reasons

- `ALL_REACHABLE_FREE_CELLS_OBSERVED`: all reachable free cells have been sensed by at least one drone.
- `STALL_30_STEPS`: no successful movement for 30 consecutive simulation steps.
- `MAX_STEPS`: maximum configured simulation horizon reached.

The global completion test uses the simulator's ground truth and the union of observations for **stopping and evaluation only**. It does not make frontier selection centralized.

## Validation and limitations

Algorithm-level smoke tests included with the prior V006 package were performed with **temporary JavaFX drawing stubs and JDK 21**; they are not substitutes for a complete JavaFX GUI or Maven build test.

| Synthetic scenario | Agents | Steps | Observed ratio | Visited coverage | Revisit | Overlap |
|---|---:|---:|---:|---:|---:|---:|
| `OverlapBenchmark.java` | 4 | 177 | 100.00% | 40.64% | 39.60% | 38.95% |
| `SixDroneSmokeTest.java` | 6 | 104 | 100.00% | 40.99% | 51.38% | 51.06% |

These are **single-scenario smoke-test observations**, not statistical findings. The six-drone overlap is still high, and the reported results **do not prove superiority over V005**. Comparison requires identical scenarios, seeds, time horizons, and baseline algorithms.

Known limitations:

- Indirect stigmergy cannot guarantee zero redundant exploration, especially in shared corridors or when locally sensed pheromone is stale or unavailable.
- ID-priority collision resolution and local backoff do not establish deadlock freedom for all obstacle layouts.
- Mission termination uses simulator-wide knowledge; a fully decentralized completion protocol is a different research problem.
- The implementation should be checked against the specific **FTFD reference** cited in the manuscript before claiming exact equivalence.
- Pheromone and distance normalization constants need calibration and sensitivity analysis.
- GUI functionality and performance should be validated in the user's complete JavaFX project.

## Reproducible evaluation roadmap

For scientific evaluation, compare (i) Nearest Frontier, (ii) frontier utility **without** pheromone (`lambda = 0`), and (iii) the proposed pheromone-aware method using the **same sensor, motion, map, and stopping protocol**. Recommended outputs include coverage-versus-time curves, revisits, multi-agent overlap, time to observed completion, waiting/replanning counts, and variance across matched random seeds.

## Repository

- Project: [FOSSALGO / Stigmergy_Repulsive_Pheromone](https://github.com/FOSSALGO/Stigmergy_Repulsive_Pheromone)
- Version folder: [V006](https://github.com/FOSSALGO/Stigmergy_Repulsive_Pheromone/tree/main/V006)

This README was prepared using the available **V006 source archive and its changelog**, since the live GitHub folder could not be retrieved for independent comparison at preparation time. If the repository has changed since the archive was produced, reconcile the documentation with the current source before publishing.

