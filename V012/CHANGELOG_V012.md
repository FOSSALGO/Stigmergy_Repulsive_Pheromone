# V012 Changelog

## Algorithmic redesign

- Restored private local occupancy maps; there is **no shared occupancy-map instance**.
- Added communication-radius-limited local map overlays at `2 × sensorFoV` and synchronized pre-merge snapshots.
- Fixed the critical known-cell export invariant: **every sensed free/occupied cell is added to the peer-export index**, not only the cell physically occupied by the drone.
- Added locally maintained frontier claim cache with time-limited leases, exact claim exclusion, a bounded neighbourhood penalty, and a liveness fallback.
- Retained global *visitation* pheromone (deposited on entered cells) and local sensor-only pheromone observations; removed the V010 territorial pheromone channel from the V012 algorithms.
- Kept independent FTFD, A*, utility maximization and minimum-ID collision arbitration.
- Added modes `LOCAL`, `LOCAL_EXCHANGE`, `EXCHANGE_CLAIM` and headless regression runner.

## Locked rendering

`CanvasView`, `MainView`, `CSS`, `SVGIcon`, `Point`, `State`, `Launcher` are unchanged from V010. Drone rendering and its 30-color palette remain unchanged.

## Testing

Single synthetic six-drone corridor smoke test: all modes reached 100% observed reachable free cells; results are reported in README. The data are preliminary and not derived from a multiparameter independent benchmark.
