# V013 — Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy

**Status:** experimental research implementation. **V013 returns to the minimal nearest-frontier principle discovered in the user-provided `SwarmDroneSimulator.java` without copying its privileged global-map assumptions.**

## What the legacy program actually does

The supplied `SwarmDroneSimulator.java` maintains one global `map`/`base_map`, uses `detectFrontiers()` to avoid entering a frontier in two drone-specific lists, and calls `new BFS().toNearestFrontier(...)` to choose nearby frontier goals. Its planning routine is given `base_map`, a copy of the actual full environment; it does **not** implement repulsive pheromone and does **not** measure revisit/overlap rigorously. Its visually cleaner paths are useful inspiration but are not proof of zero redundancy or of decentralized navigation.

## Research-preserving V013 design

1. **Private occupancy maps:** each drone knows only cells it sensed via its 360° radial sensor, or learned through range-limited peer exchange.
2. **Local peer exchange:** drones within `2 × sensorFoV` exchange snapshots of their *known* cells. Maps are never globally merged, and ground truth is inaccessible to frontier selection.
3. **Frontier tracing (FTFD-inspired incremental update):** reachable known-free cells bordering unknown cells are candidates; the implementation's incremental frontier tracing should be checked against a reference FTFD implementation before a strict bibliographic equivalence claim.
4. **Nearest-frontier-inspired utility:** BFS's nearest-target preference is reflected by increasing the **A*-path distance** penalty, while retaining expected unknown-cell gain, locally sensed repulsive pheromone, and peer frontier reservations:

   `S_i(f) = w_G * normalizedIG_i(f) - w_D * normalizedAStarDistance_i(f) - w_P * multiScalePheromone_i(f) - claimPenalty_i(f)`

   Default experimental weights: `w_G=0.25`, `w_D=1.4`, `w_P=0.45`. These are **calibration candidates**, not optimized parameters.
5. **Pheromone:** global *environmental* pheromone deposited only when a drone enters a cell, evaporating once per timestep. Each agent reads it locally through sensor range/line of sight; the pheromone field is not a globally readable occupancy map.
6. **Frontier claiming:** temporary peer target announcements prevent selection of known-claimed targets where alternatives exist; claims expire. A claimed cell is never a physical obstacle.
7. **Movement:** 8-connected A*, no obstacle corner-cutting, no pheromone added to A* edge costs. Simulator resolves contested physical cells using the smallest drone ID.
8. **Stop:** simulator terminates at 100% observed reachable free cells. This is separate from the **physically visited coverage** metric.

## Code changes and locked visualization

Only `Drone.java` changes relative to V012; `DecentralizedExploration.java` remains byte-identical to V012. `CanvasView.java`, `MainView.java`, `CSS.java`, and all remaining graphical classes remain **byte-identical to V010/V012**. All 30 fixed per-drone path colors, drone artwork, sensing overlay, explored/unexplored animation, playback, pause, replay, open, and save are preserved.

## Running from IntelliJ

Keep your existing JavaFX/Maven project, then replace **only** `Drone.java` from the package `src/main/java/swarmdrone/decentralizedswarmcoverage_ijrcs/`. To reproduce the complete reference snapshot, extract the archive in a separate directory and transfer the nine Java classes into your existing project. The package ships the Java sources and algorithm tests; it is not a complete standalone Maven distribution.

In **Run → Edit Configurations → VM Options**, select one of:

- `-Dswarm.selection=NEAREST -Dswarm.mode=EXCHANGE_CLAIM` (V013 default, proposed)
- `-Dswarm.selection=V012 -Dswarm.mode=EXCHANGE_CLAIM` (V012-style utility within the same V013 code)
- `-Dswarm.selection=NEAREST -Dswarm.mode=LOCAL_EXCHANGE` (ablation without claims)
- `-Dswarm.selection=NEAREST -Dswarm.mode=LOCAL` (private maps, no map exchange or claims)

Tune `-Dswarm.nearestDistanceWeight=1.4`, `-Dswarm.nearestGainWeight=0.25`, and `-Dswarm.nearestPheromoneWeight=0.45` only on calibration seeds; do not adjust them on held-out evaluation scenarios.

Run algorithm-focused tests on a machine with JDK 21 using:

```bash
bash tests/run_v013_comparison.sh
```

This compiles core classes against minimal JavaFX stubs, runs 4 scenario families × 2/4/6 drones × 2 selection policies, validates collision-free timeline and full sensed-area completion (or marks failed cases), and checks local map/claim invariants. **It does not compile or exercise the full JavaFX GUI**.

## Experimental cautions

- **Observed-area completion** = proportion of true reachable free cells observed by at least one drone, used by the simulation evaluator only.
- **Visited coverage** = proportion of physically entered reachable free cells.
- **Revisit rate** measures repeated cell entries; **overlap rate** measures cells physically entered by multiple distinct drones.
- Corridors can require repeat travel. A path screenshot alone cannot establish zero redundancy.
- One synthetic corridor benchmark is not enough for peer-reviewed claims; use multiple map geometries, seeds, agent numbers, consistent starting positions, and report mean/spread of coverage, completion time, revisit, and overlap.
- Local map exchange and frontier claims are **direct local communication**. The correct claim is decentralized **decision-making**, not communication-free coordination.

## Research title

**Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy**
