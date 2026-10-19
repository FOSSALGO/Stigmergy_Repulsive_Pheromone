# V007 Changelog

**Baseline:** Nine user-uploaded V006 Java classes (package `swarmdrone.decentralizedswarmcoverage_ijrcs`).

## Only modified production files

### Drone.java
- Added fixed, pure radial two-scale pheromone evaluation (`R1=1`, `R2=3`, `alpha=0.5`).
- Each neighbourhood independently averages local pheromone memory, excludes only KNOWN obstacles, retains unknown cells under A1.1 Neutral Prior, and normalizes against fixed `P_ref`.
- Added a local-memory-only observation fraction diagnostic.
- Added configurable early target utility reassessment when new occupancy cells or meaningful pheromone changes are sensed. Periodic review every 8 timesteps and switching hysteresis `0.08` remain.
- Added an event cooldown of 3 timesteps and a target review counter.
- No new route cost, pheromone diffusion, global target communication, or code changes to sensing visualizations.

### DecentralizedExploration.java
- Added 4 ablation modes via JVM `-Dswarm.multiscale` and `-Dswarm.eventReview` properties.
- Added optional `-Dswarm.regionalRadius` and `-Dswarm.localWeight` sensitivity controls.
- Validates multi-scale configuration, passes it to independent drones, and logs flags and event-review counts.
- Preserved V006 pheromone deposit/evaporation, ID-priority movement, completion criteria, and core metrics.

## Test setup / initial results

A synthetic 24 x 36 environment with corridor barriers and six drones, deterministic initial positions; tests check replay timestep synchronization, free-cell movement, collision avoidance, normalized metric bounds and multi-scale convex combination. **One scenario, not a statistically valid performance claim:**

| Mode | Steps | Observed | Revisit | Overlap |
|---|---:|---:|---:|---:|
| A0 (multi-scale off, event off) | 135 | 100% | 47.88% | 49.14% |
| A1 (multi-scale only) | 134 | 100% | 49.22% | 50.26% |
| A2 (event only) | 147 | 100% | 55.86% | 59.02% |
| A3 (multi-scale + event) | 128 | 100% | 47.32% | 46.90% |

Note that event review caused frequent target switching in this scenario; further analysis is necessary. A1/A2 did *not* consistently improve overlap. Compare multiple environments/seeds before claiming efficacy.

## Not changed

`CanvasView`, `MainView`, `CSS`, `Launcher`, `Point`, `State`, `SVGIcon`, and all existing JavaFX rendering methods.
