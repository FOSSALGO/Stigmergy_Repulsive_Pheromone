# Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy

**V005 — Pheromone-Aware Frontier Reassessment and Overlap Reduction (Experimental)**

A JavaFX-based multi-drone exploration simulator developed to investigate **decentralized swarm coverage**, **frontier-based exploration**, and **repulsive digital pheromone stigmergy** in unknown grid environments.

This directory is version **V005** of the research implementation. It introduces stronger, fixed-scale pheromone penalties and periodic reassessment of frontier targets to help reduce unnecessary trajectory overlap and revisits. It preserves independent drone decision-making, local sensing, incremental frontier tracing, and A* navigation.

> **Research status:** V005 is an **experimental implementation**, not a validated optimal solution. The improvement settings are preliminary and require sensitivity analysis, matched-budget comparisons, and evaluation across multiple environments and random seeds. A decrease in trajectory overlap is an empirical objective, **not a guarantee**.

## 1. Research approach

The simulator integrates three complementary mechanisms:

1. **Decentralized swarm coverage:** Each drone maintains an independent occupancy map and local pheromone memory, detects its own frontiers, and selects its own target. There is **no direct inter-drone map sharing or centralized frontier assignment**.
2. **Frontier-based exploration:** A frontier is a **known-free cell adjacent to at least one unknown cell**. Frontiers are maintained using an **incremental frontier-tracing / FTFD-style** procedure on each drone's local map.
3. **Repulsive pheromone stigmergy:** Drones deposit digital pheromone **only when physically entering cells**. Other drones may observe this environmental trace within their local sensor footprint. The pheromone term discourages selection of frontiers near previously traversed regions.

The simulator does centrally handle the environment, timestep synchronization, collision arbitration, and evaluation. These simulator responsibilities are distinct from **decentralized exploration decisions**.

### 1.1 Local radial sensing

- A single **360° radial sensor** (`sensorFoV`) observes free cells, obstacle cells, and locally available pheromone.
- Obstacles stop rays: cells behind obstacles remain unknown until observed from another position.
- Occupancy and pheromone observations update each drone's **private** memory.
- Rendering, pan, zoom, and playback do not alter sensor or planning coordinates.
- The legacy `cameraFoV` constructor parameter is retained for compatibility with the existing editor; it is **not used as a second mapping sensor**.

### 1.2 Frontier detection

For drone \(i\), the eight-connected frontier set is:

\[
F_i(t)=\{f\in Free_i(t)\mid N_8(f)\cap Unknown_i(t)\neq\varnothing\}.
\]

Frontier discovery and validation use the drone's locally observed occupancy map. The implementation performs an **incremental frontier-tracing update** around changed map cells; equivalence to a particular published FTFD variant should be independently verified before making a formal reproduction claim.

### 1.3 Pheromone-aware frontier utility

For each reachable frontier \(f\):

\[
S_i(f,t)=\widehat{IG}_i(f,t)-\widehat{D}_i(f,t)-\lambda\widehat{P}_i(f,t).
\]

\[
f_i^*(t)=\arg\max_{f\in F_i^{reachable}(t)} S_i(f,t).
\]

Here:

- \(\widehat{IG}_i\): expected number of previously unknown cells observable from the frontier, normalized using a sensor-based reference;
- \(\widehat D_i\): A* path distance to the frontier through **known-free** cells, normalized using a fixed reference;
- \(\widehat P_i\): average local pheromone estimate in a neighborhood of the frontier, normalized using a fixed pheromone reference;
- \(\lambda\): weight of the repulsive pheromone penalty.

**Pheromone affects frontier selection, not A* edge costs.** A* remains a geometric shortest-path planner on the locally known free-space map.

### 1.4 Environmental pheromone update

\[
P(c,t+1)=\min\!\left(P_{max},\ (1-\rho)P(c,t)+Q\sum_{i=1}^{N}v_i(c,t)\right).
\]

- \(Q\): deposit amount per physical cell entry;
- \(\rho\): per-timestep evaporation rate;
- \(P_{max}\): environmental pheromone saturation limit;
- \(v_i(c,t)\): indicator that drone \(i\) **entered** cell \(c\) at timestep \(t\).

**Sensing a cell does not deposit pheromone.** No diffusion is used in the V005 primary model. For never-observed pheromone, the local decision process uses the **A1.1 neutral prior** (zero penalty, with unknown status kept separate). An older observation is estimated to decay until refreshed by sensing.

## 2. What changed in V005?

V005 specifically addresses a weakness observed during four-drone simulation: multiple drones can continue toward the same general exploration region even after nearby pheromone levels become informative.

| Mechanism | Earlier V004-style configuration | V005 configuration |
| --- | --- | --- |
| Repulsive pheromone weight `LAMBDA` | `0.5` | **`1.0`** |
| Pheromone utility normalization | Based on `PHEROMONE_MAX = 10` | **Independent `PHEROMONE_REFERENCE = 2.0`** |
| Environmental saturation | `PHEROMONE_MAX = 10` | **Unchanged (`10`)** |
| Target reevaluation | Primarily when target becomes invalid or reached | **Every `RECHECK_INTERVAL = 8` timesteps** |
| Target-switch stability | No explicit improvement threshold | **`SWITCH_MARGIN = 0.08`** |
| Diagnostic output | Coverage, revisit, overlap | Also reports **`targetSwitches`** |

### Why distinguish pheromone reference from maximum pheromone?

In V005, the **environmental concentration limit** and the **utility normalization scale** have different functions:

\[
\widehat P_i(f,t)=\min\left(1,\frac{P_i^{frontier}(f,t)}{P_{ref}}\right),\qquad P_{ref}=2.0.
\]

Meanwhile, \(P_{max}=10\) still limits the environmental pheromone concentration. This makes recently traversed regions exert a stronger local penalty without changing the visited-cell deposit rule.

### Periodic target reassessment with hysteresis

A drone keeps a valid target unless periodic reevaluation finds a sufficiently better candidate. A new target must improve utility by more than `SWITCH_MARGIN`, reducing excessive target switching or *oscillation*.

This mechanism is **local to each drone** and does not require knowledge of the other drones' selected targets.

> The V005 parameter values are **initial experimental settings**. They should not be described as optimal without systematic calibration.

## 3. Navigation and conflict handling

- **8-connected grid movement:** axial steps cost `1`; diagonal steps cost `sqrt(2)`.
- **No diagonal corner cutting:** both adjacent orthogonal cells must be traversable.
- **A* path planning:** operates only on the drone's local known-free map.
- **Same-cell conflict:** the moving drone with the **smallest ID** receives priority.
- **Other safety checks:** prevent entry into a stationary occupied cell, head-on swaps, and crossing simultaneous movement segments.
- **Synchronous timesteps:** all drones sense and propose movements before the simulator commits movement and pheromone updates.

Collision arbitration is performed by the **simulation environment**, not by a centralized frontier-target allocator. A physical robotic deployment would require an explicit collision-avoidance and arbitration mechanism.

## 4. Project source files

Java package: `swarmdrone.decentralizedswarmcoverage`

```text
V005/
├── src/
│   └── main/java/swarmdrone/decentralizedswarmcoverage/
│       ├── Launcher.java                  # JavaFX application entry point
│       ├── MainView.java                  # UI controls and event handlers
│       ├── CanvasView.java                # Grid editor, rendering, replay, .sc I/O
│       ├── Drone.java                     # Private map, sensor, FTFD, utility, A*
│       ├── DecentralizedExploration.java  # Simulator, pheromone, collisions, metrics
│       ├── Point.java                     # Grid row/column representation
│       ├── State.java                     # Editor operation modes
│       ├── CSS.java                       # JavaFX styles
│       └── SVGIcon.java                   # Toolbar icon paths
├── README.md                              # This documentation
└── ...                                    # Other version-specific documentation
```

**Most important classes:**

| Class | Responsibility |
| --- | --- |
| `Drone` | Autonomous local sensing, occupancy mapping, pheromone memory, incremental frontier tracing, frontier utility, periodic target reassessment, A* planning, and trajectory history |
| `DecentralizedExploration` | Ground-truth simulator, time synchronization, movement-conflict resolution, environmental pheromone, global performance metrics, and playback-only observation history |
| `CanvasView` | Environment editor, radial sensor visualization, dark-to-light explored-area playback, trajectories, pan/zoom, and `.sc` file Open/Save |
| `MainView` | JavaFX toolbar and action handlers for Run, Play/Pause, Replay, Open, and Save |

> The global observed-area timeline supports **visualization only**; it must not be used by a drone when choosing a frontier.

## 5. Requirements and setup

- **JDK 21**, as used for prior core-logic compilation checks.
- **JavaFX** dependencies compatible with the existing application.
- **IntelliJ IDEA** or another Java IDE; the original working project uses a Maven-based setup.

The source distribution provides Java classes but does **not necessarily include a standalone `pom.xml` or all application resources**. To run it reliably, retain the Maven build configuration, required JavaFX modules, resource files (including the window icon), and launch setup from your existing working project.

### Running from IntelliJ IDEA

1. Open the existing working Maven project and make a backup.
2. Place the V005 Java files in `src/main/java/swarmdrone/decentralizedswarmcoverage/`.
3. Reload Maven and ensure the JavaFX dependencies resolve.
4. Launch `Launcher.main()` using your configured JavaFX run configuration (or the Maven launch goal defined by your project, if available).
5. Draw or open a test environment, add at least two drones at **distinct free cells**, and click **Run**.
6. Use **Play/Pause** and **Replay** to inspect observed and visited areas over time.

**V004 → V005 upgrade:** If your V004 project is already working, the V005 algorithm improvements were concentrated in **`Drone.java` and `DecentralizedExploration.java`**. The UI and file-handling behavior are intended to remain compatible.

## 6. User interface

| Button/tool | Purpose |
| --- | --- |
| **New** | Reset the environment and current playback |
| **Open** | Load a saved `.sc` environment and initial drone positions |
| **Save** | Save environment geometry and initial drone positions as `.sc` |
| **Transform** | Default editor interaction and initial drone positioning |
| **Add Drone** / **Remove Drone** | Add or remove a drone on the grid |
| **Wall** | Toggle an obstacle cell |
| **Draw Line** / **Remove Line** | Create or erase obstacle segments |
| **Run** | Calculate a new exploration simulation and begin its playback |
| **Play/Pause** | Pause or resume playback |
| **Replay** | Restart the **already computed** simulation playback |

The canvas supports mouse-wheel zoom and middle-mouse panning. Replaying does **not** recompute frontier decisions or pheromone dynamics; press **Run** for a fresh simulation.

### Dark-to-light exploration animation

- **Dark cells:** reachable free cells not yet *observed* by any drone at the displayed timestep.
- **Light cells:** cells already *observed* by the radial sensor(s).
- **Subtle green overlay:** cells actually *visited* by drones.
- **Red trajectory lines:** cumulative physically traveled paths up to the displayed timestep.
- **Turquoise walls:** obstacles in the simulated environment.
- **Orange regions:** current radial sensor footprints.

**Observed area \(\neq\) visited area.** Illumination indicates sensing progress, while pheromone deposits and the primary coverage/revisit/overlap metrics use physical cell visits. Trajectory crossings do not automatically mean a collision or simultaneous same-cell occupancy.

### `.sc` files

The built-in Open/Save workflow is intended to persist **environment grid geometry and drone initial positions**. It is not a full simulation-state checkpoint: playback frames, historical pheromone concentrations, and local agent memories are regenerated by pressing Run.

## 7. Experimental parameters (V005 defaults)

Most simulation parameters are declared in `DecentralizedExploration.java`:

| Parameter | Value | Meaning |
| --- | ---: | --- |
| `Q` | `1.0` | Pheromone deposit per entered cell |
| `RHO` | `0.05` | Per-timestep evaporation fraction |
| `PHEROMONE_MAX` | `10.0` | Environmental concentration cap |
| `PHEROMONE_REFERENCE` | **`2.0`** | Fixed normalization reference for frontier utility |
| `LAMBDA` | **`1.0`** | Weight of the pheromone penalty |
| `DISTANCE_REFERENCE` | `50.0` | Travel-distance normalization reference |
| `MAX_STEPS` | `1200` | Maximum simulation horizon |

The following target-management settings are declared in `Drone.java`:

| Parameter | Value | Meaning |
| --- | ---: | --- |
| `RECHECK_INTERVAL` | **`8`** | Reassess current frontier target periodically |
| `SWITCH_MARGIN` | **`0.08`** | Required utility improvement to switch targets |
| `neighbourhood` | `1` | Radius of local pheromone averaging around a frontier |

`CanvasView.java` supplies `sensorFoV = 4` cells by default. The single radial sensor uses 360 angular rays. Parameter values are **research defaults**, not validated optima.

## 8. Performance metrics

The simulator reports metrics based on **physical cell visitation**:

### Coverage ratio

\[
C(t)=\frac{\text{Unique reachable free cells visited by any drone}}{\text{Total reachable free cells}}\times100\%.
\]

### Revisit rate

\[
R(t)=\frac{\text{Total cell visits}-\text{Unique visited cells}}{\text{Total cell visits}}\times100\%.
\]

### Overlap rate

\[
O(t)=\frac{\text{Unique cells visited by at least two distinct drones}}{\text{Unique cells visited by any drone}}\times100\%.
\]

The log also reports the **number of target switches** to support diagnosis of target stability in V005.

## 9. Preliminary comparison and interpretation

A previous smoke benchmark used **four drones** in one synthetic 26 × 48 corridor environment. The results reported during development were:

| Metric | V004-style baseline | V005 |
| --- | ---: | ---: |
| Executed timesteps | 337 | 353 |
| Visited-cell coverage | 29.15% | 31.47% |
| Revisit rate | 24.50% | 21.63% |
| Overlap rate | 14.90% | 13.80% |

**Do not treat these numbers as publication-ready evidence.** They came from one developmental scenario and used **different effective numbers of timesteps**. They are not a controlled, equal-budget comparison, and this table should not be generalized to other maps.

A defensible evaluation should compare **Nearest Frontier**, **Frontier Utility without pheromone**, and **V005 pheromone-aware Frontier Utility** under identical sensing, movement, collision, and time budgets. Include multiple map topologies, repeated seeds, sensitivity analysis of `LAMBDA`/`PHEROMONE_REFERENCE`/`RHO`, and an ablation of periodic target reassessment.

## 10. Known limitations

- **Overlap can remain unavoidable** in narrow corridors, around bottlenecks, or when several drones must traverse the same connecting passages.
- Drones cannot know another drone's hidden map or intended target; only **locally sensed environmental pheromone** provides implicit coordination.
- **Observed cells and visited cells differ.** A location sensed repeatedly but never entered does not receive visited-cell pheromone deposits.
- The frontier-tracing implementation is **FTFD-style incremental boundary maintenance**; fidelity to a specific literature implementation should be confirmed by reference tests.
- Periodic target reassessment may introduce computational overhead and requires controlled ablation testing.
- The same-ID priority scheme may favor lower-ID drones in repeated traffic conflicts; deadlock and fairness need targeted testing.
- The simulator computes exploration results before playing back their recorded history; **Replay is visualization, not live re-execution**.
- The V005 improvements were tested at the **core-logic level** during development; a full JavaFX build and UI test should be conducted in the user's working environment.

## 11. Citation and research use

**Research title:**

> *Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy*

The README documents the current experimental implementation. Do not present developmental benchmark values as verified results of a published paper. When preparing a paper or supplementary material, report the exact code version, map configuration, drone initial positions, sensing settings, parameter values, simulation horizon, and trial seeds.

**Repository:** [FOSSALGO / Stigmergy_Repulsive_Pheromone — V005](https://github.com/FOSSALGO/Stigmergy_Repulsive_Pheromone/tree/main/V005)
