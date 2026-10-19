# Decentralized Swarm Coverage Using Frontier-Based Exploration and Repulsive Pheromone Stigmergy

## Implementation notes — experimental version 1.0

This repository is a **research prototype**, not a validated reproduction of published experimental results. The method and all parameter values must be independently verified before they are claimed in a paper.

### Minimal changes

- **Drone.java — algorithm rewritten:** independent occupancy map, local pheromone observations/memory, 360° obstacle-occluded radial sensing, incremental frontier tracing (FTFD-inspired implementation), independent frontier utility, A*, persistent target, one-cell movement proposal, and visited-cell trajectory. Original drone illustration and orange sensor outline preserved; the obsolete camera overlay is removed per project decision.
- **DecentralizedExploration.java — algorithm rewritten:** synchronized observation/decision/movement phases, actual environmental pheromone field, single evaporation/deposit update per tick, ID-based conflict resolution, and visited-cell metrics.
- **CanvasView.java — minimum integration:** Run retains all configured agents, eliminates frame-by-frame console spam, and adds a faint replayed visited-cell overlay. Original toolbar, obstacle rendering, pan, zoom, save/open code, colors, and drone drawing are otherwise retained.
- `MainView.java`, `CSS.java`, `Launcher.java`, `Point.java`, `State.java`, and `SVGIcon.java` retain the uploaded code.
- `pom.xml` configured for JDK 21 and correct `Launcher` entry point; icon and Maven wrapper included.

### Important algorithmic definitions

1. A drone's private occupancy map starts entirely UNKNOWN. The environment ground-truth is accessible only in the simulator's sensor operation and in the preexisting graphical sensor renderer. Decisions and A* use the private map.
2. Occupancy and pheromone are both observed through a 360-degree ray sensor of radius `sensorFoV + 0.5` cell widths, with obstacle occlusion. Ray-marching resolution is 0.25 cell, independent of UI zoom. Detection of an obstacle marks that obstacle OCCUPIED; the region beyond remains UNKNOWN.
3. FTFD is implemented as **incremental boundary tracing seeded by cells changed in the latest observation**, with a cached frontier set. This must still be compared with the precise FTFD variant cited in the paper before calling it an exact implementation of that published algorithm. Frontiers are known-free cells adjacent to an unknown cell using eight-connected adjacency.
4. The independent agent picks the highest-scoring reachable frontier using `normalized estimated gain - normalized path distance - lambda * normalized local pheromone`. Gain uses radius-based ray visibility with unknown cells optimistically considered observable until a **known** obstacle. A* uses only known-free cells, costs 1 and sqrt(2), and forbids diagonal corner cutting. Pheromone does **not** enter A* edge costs.
5. Each drone has its own observed pheromone values and observation timestamps. Previously observed pheromone is locally decayed, and **never-observed pheromone has neutral prior zero**, not an assertion of true zero. The local frontier penalty averages a radius-1 circular neighbourhood. Pheromone normalization uses a fixed Pmax.
6. **Only approved cell entries receive pheromone deposits**; the starting cell is deposited once. Evaporation and clipping occur once per simulated tick. There is no diffusion.
7. All drones propose their movements from the same timestep's observation. Collisions into a common destination are resolved in favour of the smallest ID. Stationary occupancy, head-on swaps, and interior diagonal crossings are rejected. The simulator arbitrates physical movement; it does not assign target frontiers.
8. Coverage, revisit, and overlap count **visited cells**, consistent with the manuscript's definitions, NOT merely observed cells. Thus mapping the entire reachable environment may legitimately yield visited-cell coverage lower than 100%.
9. Target persistence: keep the selected target until it is reached, invalid or the next route step becomes unavailable.

### Initial research parameters (NOT tuned/validated)

| Parameter | Value |
|---|---:|
| Sensor radius | `sensorFoV` = 4 (effective geometry = 4.5 cells) |
| Ray angles | 360, step 1° |
| Deposit Q | 1.0 |
| Evaporation rho | 0.05 |
| Pheromone cap Pmax | 10.0 |
| Repulsive penalty lambda | 0.5 |
| Pheromone neighbourhood | radius 1 cell |
| Diffusion | 0 |
| Distance reference Dref | 50 cell-cost units |
| Simulation horizon | 1200 timesteps |
| Prolonged-stall guard | 30 consecutive timesteps without any approved cell entry |

These constants are at the top of `DecentralizedExploration.java`; they are preliminary choices and should be subjected to separate calibration/validation experiments. `Dref`, the observation and gain-estimation discretization, simulation horizon, and stall guard have not been finalized in the scientific manuscript.

### JavaFX animation behaviour and limitations

The existing Run workflow remains **batch computation followed by path replay**. It computes the simulation synchronously on the JavaFX UI thread, so large maps or many frontiers can make the interface temporarily unresponsive during Run. This limitation is inherited from the original workflow. The UI's Play/Pause and Replay button handlers are still placeholders in the uploaded `MainView.java`; this release does **not** invent their behaviour. The animation timer advances through the computed trajectory automatically after Run. A green translucent overlay shows visited cells up to the replay frame, while the original red trajectory lines show the travelled path so far.

The existing original save/open text parser is not modified as it lies outside the requested algorithm changes. It has a known header-format mismatch (`writeToFile` writes an `ENVIRONMENT-----` header but `readFromFile` expects NUM ROWS first); users should not consider round-trip save/open verified in this release.

Drone visual geometry, label, propellers, and sensor stroke remain based on the original implementation; the old camera visibility overlay is removed according to the agreed single-sensor model.

### Validation performed for this package

- `Drone.java`, `Point.java`, and `DecentralizedExploration.java` compiled successfully using OpenJDK 21 with **temporary local JavaFX drawing stubs** (not the genuine JavaFX runtime).
- A small 14x14 environment passed frontier invariance against full-map reference detection, ID-priority conflict resolution, head-on swap rejection, and visited-cell accounting.
- A 18x18 and a 200x400 sparse-map scenario executed to termination without runtime exceptions in headless algorithm tests.
- **A complete JavaFX/Maven build, visual GUI inspection, and all UI regression tests have NOT been performed here** because the JavaFX/Maven dependencies are not available in this environment. Run and test on a machine with JDK 21 and Maven/network dependencies before replacing a stable repository revision.

### Run

```bash
./mvnw clean javafx:run
```

Use JDK 21 (or update both compiler and plugins consistently). Back up the current GitHub branch, review the three changed Java files, and test the UI before merging.
