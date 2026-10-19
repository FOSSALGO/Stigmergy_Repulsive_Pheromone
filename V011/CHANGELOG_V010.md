# V010 Changelog

- Rebased production files on V007 (not V008/V009).
- Added separate, per-agent decaying territorial marker layers to the environmental scheduler.
- Added local-sensor-only observation of other agents' markers, with owner's own marker excluded.
- Added a soft territorial frontier utility penalty and event-sensitive reassessment, without changing A* step cost.
- Maintained original visited-cell pheromone deposition, FTFD and end-of-observation stopping condition.
- Assigned a palette of 30 bright, distinct path colors at drone creation and file load; kept drone graphics and the rest of CanvasView unchanged.
- Added deterministic smoke tests for the original V007 behavior, territory mode, collision prevention, synchronized replay, and palette uniqueness.
- Experimental note: one tested corridor map shows small revisit/overlap reductions but slightly longer completion; results must be replicated on independent maps.
