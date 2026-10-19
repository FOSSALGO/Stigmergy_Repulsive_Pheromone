# Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy

## V008 — Experimental Algorithm Candidate

This repository version develops the **V007** six-drone exploration simulator by adding two independently switchable research mechanisms: **Path-Aware Expected Information Gain (PA-IG)** and **Progress-Aware Target Persistence (PA-TP)**. V008 is **not yet a validated improvement**: initial ablation results show that overlap may increase on corridor maps. Retain V007 for paired comparisons.

## Research architecture (unchanged)

- Every drone maintains a private ternary occupancy grid, local pheromone observations and an independent frontier target.
- A single 360° radial sensor maps free and occupied cells, observes pheromone locally, and respects obstacle occlusion.
- Incremental frontier tracing (FTFD) detects **known free** cells adjacent to unknown cells using eight-connected adjacency.
- A* computes shortest paths on the drone's **known-free** map with 8-connected movement, octile heuristic and no diagonal corner cutting.
- Visited-cell pheromone deposits and once-per-timestep environmental evaporation implement indirect stigmergic coordination.
- The simulator handles synchronized moves, collision arbitration by lowest drone ID, collective completion evaluation and replay. **Its global information is never used to choose agents' frontiers**.
- All display features (Run, Play/Pause, Replay, Open, Save, dark-to-light sensing, trajectories) are carried over without modification.

## Frontier utility

For a reachable frontier `f` of drone `i`:

```text
S_i(f,t) = normalizedGain_i(f,t)
           - normalizedDistance_i(f,t)
           - lambda * multiScalePheromonePenalty_i(f,t)
```

In V007 and V008, two Euclidean radial pheromone neighbourhoods are mixed:

```text
Pmulti = alpha * P(R1) + (1 - alpha) * P(R2)
R1 = 1 cell; R2 = 3 cells; alpha = 0.5 (experimental defaults)
```

Unobserved pheromone uses **A1.1 Neutral Prior**, i.e. an operational zero penalty, without assuming the actual environment has zero pheromone. Pheromone does **not** change A* movement costs. Route-pheromone penalty remains excluded and is not implemented in this candidate.

### B1: Path-Aware Expected Information Gain

The predicted gain counts the **unique** unknown cells that would be visible from any step along the locally planned A* path (excluding the already-sensed origin). Known obstacles occlude predicted rays; unknown cells are optimistically transparent:

```text
IG_path(f) = | Unknown_i(t) ∩ UNION_{p in A*(current,f)} predictedVisible_i(p,t) |
normalizedGain = min(1, IG_path / PATH_IG_REFERENCE)
PATH_IG_REFERENCE = 100 (provisional; fixed across candidates)
```

This uses no global map. **Caveat:** paths passing more sensing viewpoints can collect more estimated cells. Long routes can therefore score too highly despite the existing distance penalty. The fixed normalizer of 100 has NOT been calibrated and may cause saturation. These are hypotheses to test, not solved issues.

### B2: Progress-Aware Target Persistence

A valid frontier remains assigned for a **minimum of six timesteps** unless it becomes invalid, unreachable or temporarily blocked. After that minimum commitment, switching to another candidate requires an improvement over the current utility of at least `SWITCH_MARGIN` (0.08). When occupancy knowledge has improved within the preceding eight timesteps, an additional 0.025 is required. Existing periodic (8 steps) and locally triggered target reviews are unchanged.

These rules use only a drone's own newly observed occupancy cells and its locally computed frontier utility. They do not request or use the targets of other drones. Excessive commitment can worsen overlap; ablation is mandatory.

## V008 experiment configurations

Set JVM arguments (**not** application arguments):

| Mode | `-Dswarm.pathGain` | `-Dswarm.progressPersistence` | Meaning |
| --- | --- | --- | --- |
| **B0** | `false` | `false` | V007 behaviour within V008 source |
| **B1** | `true` | `false` | Path-aware gain alone |
| **B2** | `false` | `true` | Progress-aware persistence alone |
| **B3** | `true` | `true` | Both V008 mechanisms (default) |

Existing switches remain: `-Dswarm.multiscale=false` and `-Dswarm.eventReview=false`. For the B0–B3 comparison, leave these set to `true`.

## Initial controlled smoke test (six drones)

One **synthetic 24 × 36** corridor-like grid, same six starts, same parameters, deterministic conditions; not the map from the user's screenshots:

| Mode | Observed completion | Steps | Revisit | Overlap |
| --- | ---: | ---: | ---: | ---: |
| B0 | 100% | 128 | 47.32% | 46.90% |
| B1 | 100% | 117 | 48.06% | 51.44% |
| B2 | 100% | 169 | 54.40% | 58.76% |
| B3 | 100% | 150 | 57.34% | 63.47% |

**Interpretation:** neither V008 extension reduced overlap in this preliminary case. B1 completed faster but had more physical overlap; B2 and B3 worsened both overlap and completion time. No superiority claims are justified. Further calibration and paired repeated experiments are required. `observed completion` differs from physically visited coverage.

## Build and run

1. Use Java 21 (record syntax is present) and the existing JavaFX/Maven project configuration from V006/V007.
2. Put the Java source files under `src/main/java/swarmdrone/decentralizedswarmcoverage_ijrcs/` in the same package.
3. Preserve your existing `pom.xml` and `icon.png` resources.
4. Launch `Launcher` from IntelliJ IDEA or the JavaFX Maven run configuration.
5. Use **New / Open / Save** to manage grid obstacles and drone starts (`.sc`); use **Run** to compute the exploration and **Play/Pause / Replay** to inspect synchronized paths and dark-to-light sensing.
6. To run the Java-logic smoke tests (using lightweight JavaFX stubs, not full GUI tests):

```bash
bash tests/run_v008_ablation.sh
```

## File responsibilities

| Source | Role |
| --- | --- |
| `Drone.java` | Autonomous sensing, local maps, FTFD, A*, utility, PA-IG, PA-TP and movement proposal |
| `DecentralizedExploration.java` | Shared physical world and pheromone, synchronous scheduler, conflict arbitration, evaluation and ablation switches |
| `CanvasView.java` | Grid editor, sensing/trajectory rendering and synchronized replay |
| `MainView.java` | JavaFX interface and file/playback controls |
| `Point.java`, `State.java`, `CSS.java`, `SVGIcon.java`, `Launcher.java` | Supporting code, unchanged from V007 |

## Scientific evaluation guidance

Report **observed-area completion**, **physically visited coverage**, **revisit rate**, **overlap rate**, **completion time**, **total path length**, and **target-switch frequency** separately. For conclusions, perform paired-seed tests across open, corridor and cluttered environments and drone counts of 2, 4 and 6. Compare at equal observed-completion milestones (e.g. 90%, 95%, 100%) as well as equal timestep horizons. Check both the positive and negative results of each ablation.

**This implementation is a research prototype.** A single smoke test cannot validate statistical improvements or the precise literature implementation of FTFD. Full JavaFX UI testing must be performed on the real project.
