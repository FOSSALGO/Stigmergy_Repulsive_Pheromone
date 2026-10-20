package swarmdrone.decentralizedswarmcoverage_ijrcs;

import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;

public class Drone {
    // === AUTONOMOUS AGENT: private knowledge and decision making ===
    static final int UNKNOWN = 0, FREE = 1, OCCUPIED = -1;
    static final int[][] N8 = {{0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};
    private static final int RAYS = 360;
    private static final double SQRT2 = Math.sqrt(2);
    private static final double EPS = 1e-9;
    // Keep target persistence, but reconsider promptly when genuinely new
    // occupancy is sensed. No direct inter-drone communication is introduced.
    private static final int RECHECK_INTERVAL = 8;
    private static final double SWITCH_MARGIN = 0.08;
    // Used only by the existing renderer. The agent's decisions NEVER read this array.
    protected final int[][] environment;
    private final java.util.Set<Integer> frontiers = new java.util.HashSet<>();
    private final java.util.Set<Integer> changed = new java.util.HashSet<>();
    private final java.util.List<Point> route = new java.util.ArrayList<>();
    protected int initialRowPosition, initialColPosition;
    protected int rowPosition, colPosition;
    protected int sensorFoV, cellSize;
    protected double scale;
    protected String name = "";
    protected java.util.ArrayList<Point> path = new java.util.ArrayList<>();
    protected int frameIndex = -1;
    protected String visualization = "READY";
    private int id;
    private int[][] localMap;
    private double[][] localPheromone;
    private int[][] observedAt;
    private Point target;
    private int waitSteps;
    // Temporary local navigation obstacle learned from a rejected move.
    // This is collision feedback, NOT another drone's shared map or target.
    private int avoidanceCell = -1, avoidanceUntil = -1, decisionTimestep = 0;
    private boolean initialized;
    private double rho = 0.05, pMax = 10, pReference = 2.0, lambda = 1.0, distanceReference = 50;
    private int lastSelectionAt = -1;
    private int targetSwitches = 0;
    // V007: Pure radial two-scale neighbourhoods. No route pheromone cost.
    private int localRadius = 1;
    private int regionalRadius = 3;
    private double localWeight = 0.5;
    private boolean multiScaleEnabled = true;
    private boolean eventReviewEnabled = true;
    private boolean newOccupancyEvent;
    private boolean changedPheromoneEvent;
    private int lastEventReviewAt = -100;
    private int eventReassessments;
    private static final int EVENT_COOLDOWN = 3;
    private static final double PHEROMONE_EVENT_THRESHOLD = 0.30;
    private static final int NO_OBSERVATION = -1;
    // V013: legacy BFS-inspired nearest viable frontier, without using global truth.
    // Legacy source selected nearest frontier from a partitioned global list;
    // here all candidate paths must remain within PRIVATE known-free cells.
    private final String selectionMode = System.getProperty("swarm.selection", "NEAREST").toUpperCase(java.util.Locale.ROOT);
    private final double nearestDistanceWeight = Double.parseDouble(System.getProperty("swarm.nearestDistanceWeight", "1.4"));
    private final double nearestGainWeight = Double.parseDouble(System.getProperty("swarm.nearestGainWeight", "0.25"));
    private final double nearestPheromoneWeight = Double.parseDouble(System.getProperty("swarm.nearestPheromoneWeight", "0.45"));
    private final int targetLockSteps = Integer.getInteger("swarm.targetLockSteps", 12);
    // V012: private, peer-updated occupancy; no central shared map.
    private final java.util.Set<Integer> knownKeys = new java.util.HashSet<>();
    private final java.util.Map<Integer, RemoteClaim> remoteClaims = new java.util.HashMap<>();
    private boolean claimsEnabled = false;
    private int claimsReceived = 0;
    private int mapCellsReceived = 0;
    private static final int CLAIM_RADIUS = 0; // target neighbourhood, not an obstacle
    private record RemoteClaim(int cell, int validUntil) {}

    // Exact, reusable path color assigned when the agent is CREATED.
    private String pathColor = "#FF204E";
    private static final String[] PATH_COLORS = {
        "#FF204E", "#00C2FF", "#FFB000", "#7D3CFF", "#00C878",
        "#FF5E00", "#EC00B9", "#0075FF", "#A4D600", "#FF4466",
        "#00A9A5", "#FF8C00", "#9D00FF", "#20BF55", "#FF006E",
        "#00B4D8", "#D95F02", "#8F00FF", "#5BA000", "#E63946",
        "#4361EE", "#F77F00", "#7209B7", "#00B894", "#FB5607",
        "#3A86FF", "#E500A4", "#6A994E", "#F94144", "#00A6FB"
    };
    void setPathColorByIndex(int index) { pathColor = PATH_COLORS[Math.floorMod(index, PATH_COLORS.length)]; }
    String getPathColor() { return pathColor; }
    static int pathPaletteSize() { return PATH_COLORS.length; }


    public Drone(int[][] environment, int row, int col, int sensorFoV, int cameraFoV, int cellSize, double scale) {
        this.environment = environment;
        this.initialRowPosition = row;
        this.initialColPosition = col;
        this.rowPosition = row;
        this.colPosition = col;
        this.sensorFoV = sensorFoV;
        this.cellSize = cellSize;
        this.scale = scale;
        path.add(new Point(row, col));
    }

    void initializeAgent(int id, double rho, double pMax, double pReference,
                         double lambda, double distanceReference,
                         int localRadius, int regionalRadius, double localWeight,
                         boolean multiScaleEnabled, boolean eventReviewEnabled) {
        this.id = id;
        this.name = "D" + id;
        this.rho = rho;
        this.pMax = pMax;
        this.pReference = pReference;
        this.lambda = lambda;
        this.distanceReference = distanceReference;
        this.localRadius = localRadius;
        this.regionalRadius = regionalRadius;
        this.localWeight = localWeight;
        this.multiScaleEnabled = multiScaleEnabled;
        this.eventReviewEnabled = eventReviewEnabled;
        int rows = environment.length, cols = environment[0].length;
        localMap = new int[rows][cols];
        localPheromone = new double[rows][cols];
        observedAt = new int[rows][cols];
        for (int[] line : observedAt) java.util.Arrays.fill(line, -1);
        frontiers.clear();
        knownKeys.clear();
        remoteClaims.clear();
        claimsReceived = 0;
        mapCellsReceived = 0;
        changed.clear();
        route.clear();
        target = null;
        waitSteps = 0;
        lastSelectionAt = -1;
        targetSwitches = 0;
        eventReassessments = 0;
        lastEventReviewAt = -100;
        newOccupancyEvent = false;
        changedPheromoneEvent = false;
        avoidanceCell = -1;
        avoidanceUntil = -1;
        decisionTimestep = 0;
        initialized = true;
        rowPosition = initialRowPosition;
        colPosition = initialColPosition;
        path = new java.util.ArrayList<>();
        path.add(new Point(rowPosition, colPosition));
        frameIndex = -1;
        visualization = "READY";
    }

    protected void setName(String name) {
        this.name = name;
    }

    int getAgentId() {
        return id;
    }

    Point current() {
        return new Point(rowPosition, colPosition);
    }

    boolean hasFrontier() {
        return !frontiers.isEmpty();
    }

    int knownCell(int row, int col) {
        return localMap[row][col];
    }

    java.util.Set<Integer> frontierSnapshot() {
        return java.util.Set.copyOf(frontiers);
    }

    java.util.Set<Integer> changedObservationKeys() {
        return changed;
    }

    int getTargetSwitches() {
        return targetSwitches;
    }

    int getEventReassessments() {
        return eventReassessments;
    }

    // Package-private analytical hooks for deterministic tests; never global data.
    double localPheromonePenalty(Point frontier, int timestep) {
        return normalizedPheromoneAtRadius(frontier, timestep, localRadius);
    }

    double regionalPheromonePenalty(Point frontier, int timestep) {
        return normalizedPheromoneAtRadius(frontier, timestep, regionalRadius);
    }

    double multiScalePheromonePenalty(Point frontier, int timestep) {
        double local = localPheromonePenalty(frontier, timestep);
        if (!multiScaleEnabled) return local; // V006 ablation
        return localWeight * local + (1.0 - localWeight) * regionalPheromonePenalty(frontier, timestep);
    }

    double observedFraction(Point frontier, int radius) {
        int observed = 0, valid = 0;
        for (int dr = -radius; dr <= radius; dr++)
            for (int dc = -radius; dc <= radius; dc++) {
                if (dr * dr + dc * dc > radius * radius) continue;
                int r = frontier.getRow() + dr, c = frontier.getCol() + dc;
                if (!inside(r, c) || localMap[r][c] == OCCUPIED) continue;
                valid++;
                if (observedAt[r][c] != NO_OBSERVATION) observed++;
            }
        return valid == 0 ? 0 : (double) observed / valid;
    }

    private boolean inside(int r, int c) {
        return r >= 0 && c >= 0 && r < environment.length && c < environment[0].length;
    }

    private int key(int r, int c) {
        return r * environment[0].length + c;
    }

    private Point decode(int key) {
        return new Point(key / environment[0].length, key % environment[0].length);
    }

    private boolean same(Point a, Point b) {
        return a != null && b != null && a.getRow() == b.getRow() && a.getCol() == b.getCol();
    }

    /**
     * A 360-degree ray sensor observes BOTH occupancy and environmental pheromone, locally.
     */
    void observe(int[][] truth, double[][] pheromone, int timestep) {
        if (!initialized) throw new IllegalStateException("Drone not initialized");
        final double radius = sensorFoV + 0.5; // Matches existing sensor visualization radius.
        final double step = 0.25; // World units: cells. No dependence on zoom or frame rate.
        newOccupancyEvent = false;
        changedPheromoneEvent = false;
        for (int angle = 0; angle < RAYS; angle++) {
            double dx = Math.cos(Math.toRadians(angle)), dy = Math.sin(Math.toRadians(angle));
            int previous = -1;
            for (double distance = 0; distance <= radius + EPS; distance += step) {
                int c = (int) Math.floor(colPosition + 0.5 + dx * distance);
                int r = (int) Math.floor(rowPosition + 0.5 + dy * distance);
                if (!inside(r, c)) break;
                int k = key(r, c);
                if (k == previous) continue;
                previous = k;
                int status = truth[r][c] == OCCUPIED ? OCCUPIED : FREE;
                if (localMap[r][c] != status) {
                    localMap[r][c] = status;
                    changed.add(k);
                    newOccupancyEvent = true;
                    knownKeys.add(k);
                }
                // A meaningful local pheromone observation may justify early reassessment.
                // Compare to predicted stale value rather than raw last reading.
                if (observedAt[r][c] >= 0 &&
                        Math.abs(pheromone[r][c] - pheromoneAt(r, c, timestep))
                                >= PHEROMONE_EVENT_THRESHOLD) {
                    changedPheromoneEvent = true;
                }
                localPheromone[r][c] = pheromone[r][c];
                observedAt[r][c] = timestep;
                if (status == OCCUPIED) break;
            }
        }
        localMap[rowPosition][colPosition] = FREE;
        changed.add(key(rowPosition, colPosition));
        knownKeys.add(key(rowPosition, colPosition));
    }

    // The public renderer still uses path and pathColor; neither affects decisions.
    Point activeTarget() { return target == null ? null : new Point(target.getRow(), target.getCol()); }

    // Only previously OBSERVED map cells may be exported. No ground-truth leakage.
    int[] exportKnownMap() {
        int[] result = new int[knownKeys.size()];
        int i = 0;
        for (int k : knownKeys) {
            int r = k / environment[0].length, c = k % environment[0].length;
            result[i++] = localMap[r][c] == OCCUPIED ? -(k + 1) : k;
        }
        return result;
    }

    void mergePeerMap(int[] remote) {
        for (int encoded : remote) {
            int k = encoded < 0 ? -encoded - 1 : encoded;
            int r = k / environment[0].length, c = k % environment[0].length;
            if (!inside(r, c)) continue;
            int received = encoded < 0 ? OCCUPIED : FREE;
            if (localMap[r][c] == UNKNOWN) {
                localMap[r][c] = received;
                knownKeys.add(k);
                changed.add(k); // makes FTFD update the affected boundary
                newOccupancyEvent = true;
                mapCellsReceived++;
            } else if (localMap[r][c] != received) {
                throw new IllegalStateException("Contradictory peer occupancy reading at " + r + "," + c);
            }
        }
    }

    void configureClaims(boolean enabled) { claimsEnabled = enabled; }
    int getClaimsReceived() { return claimsReceived; }
    int getMapCellsReceived() { return mapCellsReceived; }
    void receivePeerClaim(int senderId, Point claimed, int timestep, int lease) {
        if (!claimsEnabled || senderId == id) return;
        if (claimed == null) {
            remoteClaims.remove(senderId);
        } else {
            remoteClaims.put(senderId, new RemoteClaim(key(claimed.getRow(), claimed.getCol()), timestep + lease));
            claimsReceived++;
        }
    }
    void expirePeerClaims(int timestep) {
        remoteClaims.entrySet().removeIf(e -> e.getValue().validUntil < timestep);
    }
    private double nearbyClaimPenalty(Point f, int timestep) {
        if (!claimsEnabled) return 0;
        double strongest = 0;
        for (RemoteClaim claim : remoteClaims.values()) {
            if (claim.validUntil < timestep) continue;
            Point p = decode(claim.cell);
            double dist = Math.hypot(p.getRow()-f.getRow(), p.getCol()-f.getCol());
            int rad = Integer.getInteger("swarm.claimRadius", 4);
            if (dist <= rad) strongest = Math.max(strongest, 1.0 - dist / (rad + 1.0));
        }
        return Double.parseDouble(System.getProperty("swarm.claimWeight", "0.9")) * strongest;
    }
    private boolean claimedByOther(Point f, int timestep) {
        if (!claimsEnabled) return false;
        for (RemoteClaim claim : remoteClaims.values()) {
            if (claim.validUntil < timestep) continue;
            Point remote = decode(claim.cell);
            if (Math.abs(remote.getRow() - f.getRow()) <= CLAIM_RADIUS &&
                Math.abs(remote.getCol() - f.getCol()) <= CLAIM_RADIUS) return true;
        }
        return false;
    }

    void updateFTFD() {
        java.util.Set<Integer> seeds = new java.util.HashSet<>();
        for (int k : changed) {
            Point p = decode(k);
            seeds.add(k);
            for (int[] d : N8)
                if (inside(p.getRow() + d[0], p.getCol() + d[1]))
                    seeds.add(key(p.getRow() + d[0], p.getCol() + d[1]));
        }
        java.util.ArrayDeque<Integer> queue = new java.util.ArrayDeque<>(seeds);
        java.util.Set<Integer> seen = new java.util.HashSet<>();
        while (!queue.isEmpty()) {
            int k = queue.removeFirst();
            if (!seen.add(k)) continue;
            Point p = decode(k);
            boolean frontierCell = isFrontier(p.getRow(), p.getCol());
            if (frontierCell) {
                frontiers.add(k);
                // Trace only the connected frontier boundary.
                for (int[] d : N8) {
                    int r = p.getRow() + d[0], c = p.getCol() + d[1];
                    if (inside(r, c) && localMap[r][c] == FREE && !seen.contains(key(r, c)))
                        queue.add(key(r, c));
                }
            } else frontiers.remove(k);
        }
        // Previously found frontier cells stay cached until an adjacent observation changes.
        changed.clear();
    }

    private boolean isFrontier(int r, int c) {
        if (!inside(r, c) || localMap[r][c] != FREE) return false;
        for (int[] d : N8) if (inside(r + d[0], c + d[1]) && localMap[r + d[0]][c + d[1]] == UNKNOWN) return true;
        return false;
    }

    private boolean traversableStep(int r, int c, int nr, int nc) {
        if (!inside(nr, nc) || localMap[nr][nc] != FREE) return false;
        if (Math.abs(nr - r) == 1 && Math.abs(nc - c) == 1) {
            // No diagonal corner-cutting, even through not-yet-observed cells.
            return inside(r, nc) && inside(nr, c) && localMap[r][nc] == FREE && localMap[nr][c] == FREE;
        }
        return true;
    }

    private double heuristic(int r, int c, int goalR, int goalC) {
        int dr = Math.abs(r - goalR), dc = Math.abs(c - goalC);
        return Math.max(dr, dc) + (SQRT2 - 1) * Math.min(dr, dc);
    }

    /**
     * A* on private known-free cells. Pheromone NEVER enters movement cost.
     */
    private java.util.List<Point> astar(Point goal) {
        int count = environment.length * environment[0].length;
        double[] cost = new double[count];
        java.util.Arrays.fill(cost, Double.POSITIVE_INFINITY);
        int[] parent = new int[count];
        java.util.Arrays.fill(parent, -1);
        java.util.PriorityQueue<Node> open = new java.util.PriorityQueue<>(java.util.Comparator.comparingDouble(n -> n.priority));
        int source = key(rowPosition, colPosition), finish = key(goal.getRow(), goal.getCol());
        cost[source] = 0;
        open.add(new Node(source, heuristic(rowPosition, colPosition, goal.getRow(), goal.getCol())));
        while (!open.isEmpty()) {
            Node n = open.poll();
            Point p = decode(n.key);
            if (n.priority > cost[n.key] + heuristic(p.getRow(), p.getCol(), goal.getRow(), goal.getCol()) + EPS)
                continue;
            if (n.key == finish) {
                java.util.LinkedList<Point> answer = new java.util.LinkedList<>();
                for (int k = finish; k != -1; k = parent[k]) answer.addFirst(decode(k));
                return answer;
            }
            for (int[] d : N8) {
                int nr = p.getRow() + d[0], nc = p.getCol() + d[1];
                if (!traversableStep(p.getRow(), p.getCol(), nr, nc)) continue;
                int k = key(nr, nc);
                if (k == avoidanceCell && decisionTimestep < avoidanceUntil) continue;
                double g = cost[n.key] + (d[0] != 0 && d[1] != 0 ? SQRT2 : 1);
                if (g + EPS < cost[k]) {
                    cost[k] = g;
                    parent[k] = n.key;
                    open.add(new Node(k, g + heuristic(nr, nc, goal.getRow(), goal.getCol())));
                }
            }
        }
        return java.util.List.of();
    }

    private double routeDistance(java.util.List<Point> candidate) {
        double distance = 0;
        for (int i = 1; i < candidate.size(); i++) {
            Point a = candidate.get(i - 1), b = candidate.get(i);
            distance += a.getRow() != b.getRow() && a.getCol() != b.getCol() ? SQRT2 : 1;
        }
        return distance;
    }

    /**
     * Expected visible unknown cells from frontier; known obstacles occlude rays.
     */
    private int expectedGain(Point f) {
        java.util.Set<Integer> unknown = new java.util.HashSet<>();
        double radius = sensorFoV + 0.5;
        for (int angle = 0; angle < RAYS; angle += 2) {
            double dx = Math.cos(Math.toRadians(angle)), dy = Math.sin(Math.toRadians(angle));
            int last = -1;
            for (double distance = 0; distance <= radius + EPS; distance += 0.5) {
                int c = (int) Math.floor(f.getCol() + .5 + dx * distance);
                int r = (int) Math.floor(f.getRow() + .5 + dy * distance);
                if (!inside(r, c)) break;
                int k = key(r, c);
                if (k == last) continue;
                last = k;
                if (localMap[r][c] == OCCUPIED) break;
                if (localMap[r][c] == UNKNOWN) unknown.add(k);
            }
        }
        return unknown.size();
    }

    private double pheromoneAt(int r, int c, int timestep) {
        if (!inside(r, c) || observedAt[r][c] < 0) return 0; // A1.1 zero-penalty prior.
        return localPheromone[r][c] * Math.pow(1 - rho, timestep - observedAt[r][c]);
    }

    /**
     * V007: fixed-radius Euclidean disk, including UNKNOWN cells in the denominator.
     * Never-observed cells contribute zero (A1.1 neutral prior); ONLY locally
     * observed pheromone can contribute a positive value.
     * Known obstacles and out-of-bounds cells are excluded.
     */
    private double normalizedPheromoneAtRadius(Point f, int t, int radius) {
        double sum = 0;
        int count = 0;
        for (int dr = -radius; dr <= radius; dr++)
            for (int dc = -radius; dc <= radius; dc++) {
                if (dr * dr + dc * dc > radius * radius) continue;
                int r = f.getRow() + dr, c = f.getCol() + dc;
                if (!inside(r, c) || localMap[r][c] == OCCUPIED) continue;
                sum += pheromoneAt(r, c, t);
                count++;
            }
        if (count == 0) return 0;
        return Math.min(1.0, (sum / count) / pReference);
    }

    /**
     * Each drone selects independently using its private map and locally sensed pheromone.
     * Peer occupancy and claim data arrive only through explicit short-range exchange.
     * A* routes are calculated only over KNOWN FREE cells.
     */
    Point propose(int timestep) {
        decisionTimestep = timestep;
        if (target != null && (!isFrontier(target.getRow(), target.getCol()) || route.isEmpty())) {
            target = null;
            route.clear();
        }
        if (target != null && route.size() > 1) {
            Point next = route.get(1);
            if (!traversableStep(rowPosition, colPosition, next.getRow(), next.getCol())) {
                target = null;
                route.clear();
            }
        }

        // A newly observed map cell can invalidate the desirability of the
        // previous goal well before the periodic timer expires.
        // Retain periodic reassessment to prevent excessive target oscillation.
        // Newly sensed map data still invalidate a target immediately above.
        boolean eventReview = eventReviewEnabled && target != null &&
                (newOccupancyEvent || changedPheromoneEvent) &&
                timestep - lastEventReviewAt >= EVENT_COOLDOWN;
        boolean review = target == null || (target != null && claimedByOther(target, timestep))
                         || (timestep - lastSelectionAt >= RECHECK_INTERVAL)
                         || eventReview;
        if (review) {
            if (eventReview) {
                eventReassessments++;
                lastEventReviewAt = timestep;
            }

            double bestScore = Double.NEGATIVE_INFINITY;
            double fallbackScore = Double.NEGATIVE_INFINITY;
            Point fallbackTarget = null;
            java.util.List<Point> fallbackPath = java.util.List.of();
            Point bestTarget = null;
            java.util.List<Point> bestPath = java.util.List.of();
            double currentScore = Double.NEGATIVE_INFINITY;
            java.util.List<Point> currentPath = java.util.List.of();
            double radius = sensorFoV + 0.5;
            int maxGain = 0;
            for (int dr = -(sensorFoV + 1); dr <= sensorFoV + 1; dr++)
                for (int dc = -(sensorFoV + 1); dc <= sensorFoV + 1; dc++)
                    if (Math.hypot(dr, dc) <= radius) maxGain++;

            java.util.List<Integer> ordered = new java.util.ArrayList<>(frontiers);
            java.util.Collections.sort(ordered);
            for (int k : ordered) {
                Point f = decode(k);
                if (!isFrontier(f.getRow(), f.getCol())) continue;
                java.util.List<Point> candidate = astar(f);
                if (candidate.size() <= 1) continue;
                int predictedNewCells = expectedGain(f);
                // No incentive to travel toward a frontier that provides NO
                // expected observations, even if the frontier predicate holds.
                if (predictedNewCells <= 0) continue;
                double gain = Math.min(1.0, (double) predictedNewCells / Math.max(1, maxGain));
                double distance = Math.min(1.0, routeDistance(candidate) / distanceReference);
                // Fixed reference is independent of Pmax (which only limits saturation).
                double pheromone = multiScalePheromonePenalty(f, timestep);
                // V013 uses nearest-reachable-frontier bias from the user's
                // SwarmDroneSimulator BFS. IG and repulsive pheromone remain
                // explicit criteria, unlike the legacy ground-truth BFS.
                double score = "NEAREST".equals(selectionMode)
                        ? nearestGainWeight * gain - nearestDistanceWeight * distance
                            - nearestPheromoneWeight * pheromone - nearbyClaimPenalty(f, timestep)
                        : gain - distance - lambda * pheromone - nearbyClaimPenalty(f, timestep);
                if (same(target, f)) {
                    currentScore = score;
                    currentPath = candidate;
                }
                // Soft reservation: do not SELECT claimed frontiers if alternatives exist.
                // This does not prohibit traversing such cells with A*.
                if (claimedByOther(f, timestep)) {
                    if (score > fallbackScore + EPS) {
                        fallbackScore = score;
                        fallbackTarget = f;
                        fallbackPath = candidate;
                    }
                    continue;
                }
                if (score > bestScore + EPS ||
                        (Math.abs(score - bestScore) <= EPS &&
                                bestTarget != null && routeDistance(candidate) < routeDistance(bestPath) - EPS)) {
                    bestScore = score;
                    bestTarget = f;
                    bestPath = candidate;
                }
            }

            // Liveness fallback: when every useful frontier is claimed, exploration
            // must not stop indefinitely (claims may also have become stale).
            if (bestTarget == null && fallbackTarget != null) {
                bestTarget = fallbackTarget;
                bestPath = fallbackPath;
                bestScore = fallbackScore;
            }
            if (bestTarget != null) {
                // A still-valid target is retained unless a distinctly better
                // option appears; pheromone changes can now change a committed goal.
                // Give a committed drone time to reach its selected frontier;
                // otherwise simultaneous map exchange causes oscillating targets.
                boolean lockActive = "NEAREST".equals(selectionMode) && target != null &&
                        !currentPath.isEmpty() && !claimedByOther(target, timestep) &&
                        timestep - lastSelectionAt < targetLockSteps;
                if (target == null || currentPath.isEmpty() || claimedByOther(target, timestep) ||
                        (!lockActive && bestScore > currentScore + SWITCH_MARGIN)) {
                    if (target != null && !same(target, bestTarget)) targetSwitches++;
                    target = bestTarget;
                    route.clear();
                    route.addAll(bestPath);
                } else {
                    route.clear();
                    route.addAll(currentPath);
                }
            } else {
                target = null;
                route.clear();
            }
            lastSelectionAt = timestep;
        }
        return route.size() > 1 ? route.get(1) : current();
    }

    /**
     * Apply an approved move; a rejected proposal never deposits pheromone.
     */
    boolean move(Point approved) {
        if (same(approved, current())) {
            // Three consecutive movement denials indicate local congestion.
            // Temporarily avoid the rejected NEXT cell and replan via A*.
            if (route.size() > 1 && ++waitSteps >= 3) {
                avoidanceCell = key(route.get(1).getRow(), route.get(1).getCol());
                avoidanceUntil = decisionTimestep + 8;
                route.clear();
                target = null;
                waitSteps = 0;
            }
            return false;
        }
        waitSteps = 0;
        if (!traversableStep(rowPosition, colPosition, approved.getRow(), approved.getCol())) return false;
        rowPosition = approved.getRow();
        colPosition = approved.getCol();
        if (route.size() > 1 && same(route.get(1), approved)) route.remove(0);
        else {
            route.clear();
            target = null;
        }
        if (target != null && same(target, current())) {
            target = null;
            route.clear();
        }
        return true;
    }

    /**
     * One recorded position per GLOBAL timestep, including stationary frames.
     */
    void recordFrame() {
        path.add(current());
    }

    /**
     * Old visualization is replayed from computed trajectory after Run.
     */
    protected void update() {
        if (!"PLAY".equals(visualization) || path.isEmpty()) return;
        if (frameIndex < path.size() - 1) frameIndex++;
    }

    protected void draw(GraphicsContext gc) {
        int renderRow = rowPosition, renderCol = colPosition;
        if (frameIndex < 0 || path == null || path.isEmpty() || frameIndex >= path.size()) {
            renderRow = initialRowPosition;
            renderCol = initialColPosition;
        } else {
            renderRow = path.get(frameIndex).getRow();
            renderCol = path.get(frameIndex).getCol();
        }

        double halfCellSize = 0.5 * cellSize;
        double droneX = (double) renderCol * cellSize + halfCellSize;
        double droneY = (double) renderRow * cellSize + halfCellSize;
        if (droneX >= 0 && droneY >= 0) {
            gc.setFill(Color.valueOf("#FF7D29"));
            drawSensoreVisibility(gc, droneX, droneY);

            // gambar drone
            gc.setFill(Color.BLACK);
            gc.fillRect(droneX - 1.5, droneY - 1.5, 3, 3);
            gc.setLineWidth(0.4);
            gc.setStroke(Color.BLACK);
            double rx1 = droneX - 4;
            double ry1 = droneY - 4;
            double rx2 = droneX + 4;
            double ry2 = droneY + 4;
            double rx3 = droneX - 4;
            double ry3 = droneY + 4;
            double rx4 = droneX + 4;
            double ry4 = droneY - 4;
            gc.strokeLine(rx1, ry1, rx2, ry2);
            gc.strokeLine(rx3, ry3, rx4, ry4);
            //propeller
            double rPropeller = 3;
            gc.strokeOval(rx1 - rPropeller, ry1 - rPropeller, 2 * rPropeller, 2 * rPropeller);
            gc.strokeOval(rx2 - rPropeller, ry2 - rPropeller, 2 * rPropeller, 2 * rPropeller);
            gc.strokeOval(rx3 - rPropeller, ry3 - rPropeller, 2 * rPropeller, 2 * rPropeller);
            gc.strokeOval(rx4 - rPropeller, ry4 - rPropeller, 2 * rPropeller, 2 * rPropeller);

            // draw drone name
            gc.setGlobalAlpha(1.0);
            gc.setFont(Font.font("Arial", 8));
            //gc.setFill(Color.WHITE);
            gc.setFill(Color.valueOf("#FF0087"));
            gc.fillText(name + " (" + (renderRow + 1) + "," + (renderCol + 1) + ")", droneX + cellSize, droneY - cellSize);
        }
    }

    protected void drawSensoreVisibility(GraphicsContext gc, double sx, double sy) {
        int numRows = environment.length;
        int numCols = environment[0].length;
        int rays = 360;
        double[] xPoints = new double[rays];
        double[] yPoints = new double[rays];
        double cSize = cellSize;//tidak perlu dikalikan dg scale karena dia dipanggil di dalam method render setelah mengatur ulang scale
        double hcSize = 0.5 * cSize;

        for (int i = 0; i < rays; i++) {
            double angle = Math.toRadians(i);
            double dx = Math.cos(angle);
            double dy = Math.sin(angle);

            double x = sx;
            double y = sy;
            double step = cSize * 0.20; // visual-only sampling; same radial geometry
            double checked = 0;
            double MAX_RADIUS = (double) sensorFoV * cSize + hcSize;
            while (checked < MAX_RADIUS) {
                x += dx * step;
                y += dy * step;
                checked += step;
                int col = (int) (x / cSize);
                int row = (int) (y / cSize);
                if (row < 0 || row >= numRows || col < 0 || col >= numCols || environment[row][col] == -1) {
                    break;
                }
            }
            xPoints[i] = x;
            yPoints[i] = y;

        }
        gc.setGlobalAlpha(0.3);
        gc.fillPolygon(xPoints, yPoints, rays);
        gc.setGlobalAlpha(1.0);
    }

    private record Node(int key, double priority) {
    }


}
