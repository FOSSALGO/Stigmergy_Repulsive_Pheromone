# V007 — Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy

Research prototype in Java/JavaFX. **V006 remains the experimental baseline**. V007 implements an experimental **Multi-Scale Repulsive Pheromone Coordination** strategy with an optional event-triggered target reassessment. This is a *hypothesis to test*, not an established optimal improvement.

> **Hypothesis:** Multi-scale evaluation of locally observed repulsive pheromone can reduce redundant exploration by incorporating spatial visitation history at both local and regional scales, while preserving decentralized frontier selection and communication-free swarm coordination.

## What changed from V006?

Only two algorithm files were edited:

- `Drone.java` — implements two pure Euclidean-radius neighbourhoods around each frontier and event-triggered reassessment, with target-switching hysteresis.
- `DecentralizedExploration.java` — provides experimental defaults and switches for ablation; logs parameters and event reassessment counts.

Seven other classes are copied from the **user-uploaded V006 source** unchanged, including `CanvasView.java` and `MainView.java`. The drone illustration, dark-to-light explored-area playback, paths, and New/Open/Save/Run/Play-Pause/Replay controls are unchanged. **No route-pheromone penalty is used in the main V007 model** (`gamma = 0`), and pheromone does not modify A* costs.

## Multi-scale penalty

For agent `i`, frontier `f`, and radius `R`, define a Euclidean disk:

```text
N_R(f) = {c in grid : EuclideanDistance(c,f) <= R}
```

For each neighbourhood, cells outside the map and cells already **known** as obstacles are excluded. UNKNOWN cells remain in the denominator and contribute a zero penalty under **A1.1 Neutral Prior**; no ground-truth obstacle information enters a drone's penalty computation.

```text
P_i^R(f,t) = SUM_{c in valid N_R(f)} localPheromoneEstimate_i(c,t)
             ------------------------------------------------------
                        |valid N_R(f)|

P_i^R_normalized(f,t) = min(1, P_i^R(f,t) / P_ref)

P_i^multi(f,t) = alpha * P_i^R1_normalized(f,t)
                 + (1-alpha) * P_i^R2_normalized(f,t)
```

Main frontier utility:

```text
S_i(f,t) = normalizedExpectedInformationGain_i(f,t)
           - normalizedAStarTravelDistance_i(f,t)
           - lambda * P_i^multi(f,t)
```

The final frontier choice is made by each drone independently, using its local occupancy map and locally observed pheromone memory. The environmental pheromone **still deposits on physically visited cells only**, evaporates once per timestep, and is never read globally by decision-making agents.

## Preliminary configuration

| Parameter | Default | Meaning |
|---|---:|---|
| `R1` (`LOCAL_RADIUS`) | 1 cell | Local disk around a frontier |
| `R2` (`REGIONAL_RADIUS`) | 3 cells | Larger disk around a frontier |
| `alpha` (`LOCAL_WEIGHT`) | 0.5 | Relative local contribution |
| `lambda` | 1.0 | Repulsive pheromone weight |
| `P_ref` | 2.0 | Fixed pheromone normalizer |
| `P_max` | 10.0 | Environmental pheromone saturation |
| `Q` | 1.0 | Deposit per physical cell entry |
| `rho` | 0.05 | Evaporation per timestep |
| `D_ref` | 50.0 | Fixed distance reference |
| Target review interval | 8 timesteps | V006 periodic review |
| Target switch margin | 0.08 | Hysteresis |
| Event review cooldown | 3 timesteps | Caps additional reviews |
| Significant pheromone change | 0.30 raw units | Event threshold |

**All numerical values are research defaults, not tuned optima.** The V006 FTFD implementation, 360-ray sensor, 8-connected movement, no corner-cutting, A*, ID-priority collision resolution, local congestion recovery, and mission-completion termination are retained.

## Ablation options

Four modes can be tested from the same compiled project using JVM system properties, with no source changes:

```text
-Dswarm.multiscale=false -Dswarm.eventReview=false  # A0: V006-style logic
-Dswarm.multiscale=true  -Dswarm.eventReview=false  # A1: multi-scale only
-Dswarm.multiscale=false -Dswarm.eventReview=true   # A2: event-review only
-Dswarm.multiscale=true  -Dswarm.eventReview=true   # A3: combined (V007 default)
```

Optional multi-scale sensitivity parameters:

```text
-Dswarm.regionalRadius=4
-Dswarm.localWeight=0.75
```

`regionalRadius` must be strictly larger than the local radius 1; `localWeight` must be in [0,1]. Restart the Java application between configurations; these settings are read at class initialization. For a pure V006 reference, run the **original V006** separately too: ablation A0 is designed to preserve its choice rules, but it is not a substitute for verifying historical version equivalence.

**Route-pheromone evaluation is deliberately not implemented** in V007. It remains an optional *future ablation* and should not silently be represented as an implemented feature.

## Installation

The source package remains `swarmdrone.decentralizedswarmcoverage_ijrcs` (as in uploaded V006).

1. Back up V006.
2. Use the existing V006 IntelliJ/Maven project and dependencies. This ZIP contains the nine Java source classes, not a newly authored Maven `pom.xml` or packaged resources.
3. Replace **only** `Drone.java` and `DecentralizedExploration.java` from `src/main/java/swarmdrone/decentralizedswarmcoverage_ijrcs/`.
4. Keep all existing JavaFX dependencies, `icon.png`, and remaining source files.
5. Run the project's existing `Launcher`/`MainView` configuration.
6. Open a `.sc` map or draw obstacles, add drones, and click Run. Use Play/Pause and Replay to inspect the resulting observed and visited cells.

## Experiment and evaluation

The simulator's global ground truth is allowed to calculate an **evaluation-only** observed-area union and stop when reachable free cells have all been sensed. This information is not provided to individual drone target selection. Keep distinct metrics:

- `observed-area ratio`: union of sensed reachable free cells / reachable free cells, used for stopping;
- `visited coverage`: unique physically visited reachable free cells / reachable free cells;
- `revisit rate`: repeated physical cell visits / total physical cell visits;
- `overlap rate`: unique cells visited by >=2 different drones / all unique visited cells;
- `completion timesteps`, target switches, and event review counts.

Compare matched maps, starting positions, sensor settings, and seeds. Evaluate revisit and overlap both at common timesteps and at matched observed-area completion. Corridor, open-area, and cluttered-environment tests are needed before drawing conclusions.

### Core-only smoke tests

`tests/V007SmokeTest.java` and small JavaFX compile stubs allow **core algorithm** compilation/testing without native JavaFX libraries. See `tests/run_core_tests.sh`. These stubs are for tests **only**, never for the actual GUI project.

## Known limitations

- Multiple agents can still explore similar regions because they do not communicate maps, positions, targets, or exploration intent.
- Multi-scale penalty may underweight sparsely observed regional neighbourhoods under the A1.1 neutral prior; `observedFraction(frontier,radius)` is included as a diagnostic helper.
- Required corridor transits cannot always be avoided, and stronger repulsion can increase travel distance or overlap.
- Event-triggered reassessment can increase target switches; this must be evaluated, not assumed beneficial.
- Current V006-style incremental frontier tracing should be checked against the particular FTFD reference cited in the paper before claiming literal equivalence.
- Visual GUI execution still requires testing in the user's JavaFX/IntelliJ project. Core Java compilation/tests alone do not verify the complete JavaFX GUI.

## Research status

**V007 experimental implementation; not yet validated for publication claims.** See `CHANGELOG_V007.md` for detailed code changes and the paired-mode smoke-test outcomes.
