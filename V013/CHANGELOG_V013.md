# V013 Change Log

- Returned to V012 architecture and locked V010 graphical sources.
- Studied `SwarmDroneSimulator.java`: actual method is global-map-assisted nearest-frontier BFS with globally unique frontier lists, not communication-free decentralized pheromone coverage.
- Adopted **nearest reachable frontier bias**, retaining per-agent occupancy maps, FTFD-style updates, local map exchange, expiring claims, A* over known free cells, multi-scale repulsive pheromone, and synchronous ID-priority collision resolution.
- Added `swarm.selection=NEAREST|V012` and configurable distance/IG/pheromone weights for controlled ablative comparisons.
- Reduced rapid target changes through a small commitment window while keeping invalidation, congestion handling and claims.
- Added `V013Scenarios.java` and `run_v013_comparison.sh` for four geometries and 2/4/6 agents.
- No production Java UI or simulator class edits. Changes are confined to `Drone.java`.
- Results are calibration observations, not guarantees of zero revisit or overlap.
