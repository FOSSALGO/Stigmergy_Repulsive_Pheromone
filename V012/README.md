# V012 — Decentralized Swarm Coverage with Local Map Exchange and Repulsive Digital Pheromone

**Paper:** *Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy*

V012 redesigns the coordination mechanism using **private per-drone occupancy maps**, **local peer-to-peer map overlays**, and **temporarily leased frontier claims**. A **single environment-level visitation pheromone grid** is retained, but pheromone values are available to drones **only through their local 360° sensor observations**. No drone accesses a global occupancy map or the entire pheromone field for its decisions.

## Core mechanisms

1. **Radial sensing:** `sensorFoV` (default 4 cells), obstacle occlusion. A drone updates *its own* occupancy map with sensed free/occupied cells.
2. **Local map exchange:** when two drones are at most `2 × sensorFoV` cells apart, they exchange known occupancy readings. Both maps remain private and separately allocated. Knowledge received from another drone may subsequently be relayed through an encounter; no shared-map singleton exists. Map exchanges use immutable snapshots taken before any merges within a timestep.
3. **FTFD:** an incremental frontier update checks known-free cells adjoining unknown cells in an 8-connected grid. Received occupancy changes are included in the FTFD update queue.
4. **Repulsive pheromone:** a physical visit (not a sensor observation) deposits `Q` into an environment grid, which evaporates once per timestep. Each drone sees pheromone only under its own sensor footprint; the frontier utility includes a local-memory-based two-scale repulsive term.
5. **Temporary claims:** nearby drones exchange last-timestep claimed frontier coordinates. A peer's **exact claimed frontier** is excluded from the preferred candidate set, and nearby frontier candidates receive a soft penalty. Claims have a configurable lease and expire when not refreshed. If all remaining reachable frontiers are claimed, a fallback is allowed to prevent a deadlock. This is **not an absolute global reservation**, because disconnected drones cannot know one another's claims. Claim markers are not obstacles for A* transit.
6. **Independent decisions:** each drone independently chooses its frontier using expected information gain, known-free A* distance, locally sensed pheromone and locally received claims. No central allocator assigns targets.
7. **Global simulator evaluation only:** observed reachable free cells, visited coverage, revisit and inter-drone overlap. These global metrics are never fed back into drone decisions.

### Simplified decision model

\[
S_i(f)=\widehat{IG}_i(f)-\widehat D_i(f)-\lambda\widehat P_i^{multi}(f)-C_i(f)
\]

Here `C_i(f)` is a bounded soft penalty generated only from the *locally received* claims of other drones. Exact claimed targets are preferably excluded. The formula does **not** add pheromone to A* path costs.

Environmental pheromone update:

\[
P(c,t+1)=\min\left(P_{max},(1-\rho)P(c,t)+Q\sum_i v_i(c,t)\right).
\]

Only physical cell entries count in `v_i`; waiting does not generate a deposit.

## Experiment modes

Use Java VM option `-Dswarm.mode=...`:

| Mode | Private maps | Local exchange | Frontier claims | Visitation pheromone |
|---|---|---|---|---|
| `LOCAL` | Yes | No | No | Yes |
| `LOCAL_EXCHANGE` | Yes | Yes | No | Yes |
| `EXCHANGE_CLAIM` (default) | Yes | Yes | Yes | Yes |

Relevant optional VM properties:

- `-Dswarm.claimLease=8` — remote claim lifetime after receipt, in timesteps.
- `-Dswarm.claimRadius=4` — radius for the **soft neighbouring-target penalty**; exact-claim exclusion remains coordinate-based.
- `-Dswarm.claimWeight=0.9` — soft claim penalty weight.
- `-Dswarm.regionalRadius=3`, `-Dswarm.localWeight=0.5` — two-scale pheromone evaluation.
- `-Dswarm.eventReview=true` — existing event-driven target reassessment.

These are **initial experimental settings, not optimized values**.

## Preserved visualization (V010 locked)

All 7 non-algorithm Java classes, including `CanvasView.java` and `MainView.java`, are **byte-identical to V010**. The UI still provides New, Open, Save, Run, Play/Pause, Replay, 360° orange sensing overlays, dark-to-light observed area, and **30 distinct drone path colors assigned on creation and restored when opening `.sc` environments**. `Drone.draw()` also preserves the same rendering behavior. Playback remains synchronized per timestep.

## Project layout

- `src/main/java/swarmdrone/decentralizedswarmcoverage_ijrcs/Drone.java` — autonomous local map, sensing, FTFD, A*, pheromone utility, local map exchange and claim memory.
- `src/main/java/swarmdrone/decentralizedswarmcoverage_ijrcs/DecentralizedExploration.java` — environmental pheromone, encounter/message scheduling, movement arbitration, metrics and replay data.
- `src/main/java/swarmdrone/decentralizedswarmcoverage_ijrcs/CanvasView.java` — unmodified V010 renderer/editor.
- `tests/run_v012_ablation.sh` — headless Java logic test for three modes.

### Installation

Backup V010, then replace **only `Drone.java` and `DecentralizedExploration.java`** in the existing IntelliJ/JavaFX Maven project. Keep the existing POM, resources and UI files. Run your existing `Launcher` configuration with the desired VM option.

To run headless logic tests on Linux/macOS or Git Bash (JDK 21):

```bash
bash tests/run_v012_ablation.sh
```

The headless tests use minimal JavaFX test stubs to verify the core algorithm; they do **not** establish complete JavaFX GUI compatibility on every workstation.

## Preliminary six-drone results

A single deterministic **synthetic corridor environment** (24 × 36 grid, fixed start cells) produced the following results, with 100% observed reachable free cells in every mode:

| Mode | Timesteps | Revisit rate | Overlap rate |
|---|---:|---:|---:|
| LOCAL | 128 | 47.32% | 46.90% |
| LOCAL_EXCHANGE | 94 | 46.93% | 44.90% |
| EXCHANGE_CLAIM | 86 | 41.48% | 36.99% |

These are **one-scenario smoke-test results**, not a general performance claim or statistical validation. The screenshot environment supplied by the researcher has not been run from its native `.sc` file in this package. The effects must be checked across corridor/open/cluttered environments, initial placements, drone counts (2/4/6), and paired random seeds. Compare metrics at matched observation completion, not merely from the path visualization.

## Limitations and research integrity

- Map sharing makes the implementation **communication-assisted decentralized decision-making**, not communication-free stigmergy.
- All drones use the simulator's common grid coordinate frame; real deployment would require map alignment and a communication protocol.
- Remote claims do not guarantee global exclusivity outside radio range. A liveness fallback also permits a claimed frontier when every candidate is claimed; this is intentional to avoid permanent unexplored regions.
- The global environmental pheromone is used as a **stigmergic medium**, but only local sensor readings enter a drone's frontier decision.
- The existing FTFD-like incremental boundary method must be checked against the exact FTFD literature implementation before making an equivalence claim.
- Avoid claiming improvements in the manuscript before matched, independent experiments establish them.
