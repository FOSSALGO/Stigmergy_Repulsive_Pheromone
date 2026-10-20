package swarmdrone.decentralizedswarmcoverage_ijrcs;

import java.util.*;

/**
 * Legacy SwarmDroneSimulator-inspired exploration benchmark.
 * Features intentionally reproduced: a single globally updated occupancy map,
 * exclusive ownership of detected frontier cells, and nearest-frontier travel.
 * The original program gave path planning access to ground-truth base_map; this
 * benchmark deliberately retains that oracle navigation assumption to make a
 * fair conceptual comparison with the uploaded legacy algorithm. It is NOT a
 * decentralized algorithm and does NOT implement repulsive pheromone.
 * <p>
 * Changes from legacy: A* with Euclidean edge lengths instead of BFS step count,
 * simultaneous one-cell movement, collision resolution, and replay-compatible
 * observation timestamps. No corner-cutting through obstacles.
 */
public final class DecentralizedExploration {
    private static final int MAX_STEPS = Integer.getInteger("swarm.legacy.maxSteps", 2500);
    private static final int[][] N8 = {{0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};
    private static final double SQRT2 = Math.sqrt(2.0);
    private final int[][] truth;
    private final ArrayList<Drone> drones;
    private final int rows, cols;
    private final int[][] map; // 0=unknown, 1=known free, -1=known obstacle
    private final int[][] firstObservedAt;
    private final int[][] visits;
    private final BitSet[] visitors;
    private final boolean[][] reachable;
    private final int[][] owner; // per-cell exclusive frontier owner, -1 if unassigned
    private final Point[] goals;
    private final List<Point>[] routes;
    private final int[] row, col;
    private final int reachableFree;
    private final double coverage;
    private final double revisit;
    private final double overlap;
    private int observedReachable;
    private int steps;
    private int uniqueVisits;
    private int totalVisits;
    private String terminationReason = "NOT_STARTED";

    @SuppressWarnings("unchecked")
    public DecentralizedExploration(int[][] environment, ArrayList<Drone> drones) {
        this.truth = environment;
        this.drones = drones;
        rows = environment.length;
        cols = environment[0].length;
        map = new int[rows][cols];
        owner = new int[rows][cols];
        firstObservedAt = new int[rows][cols];
        visits = new int[rows][cols];
        visitors = new BitSet[rows * cols];
        reachable = new boolean[rows][cols];
        goals = new Point[drones.size()];
        routes = new List[drones.size()];
        row = new int[drones.size()];
        col = new int[drones.size()];
        for (int r = 0; r < rows; r++) {
            Arrays.fill(firstObservedAt[r], -1);
            Arrays.fill(owner[r], -1);
        }
        for (int i = 0; i < drones.size(); i++) {
            Drone d = drones.get(i);
            row[i] = d.initialRowPosition;
            col[i] = d.initialColPosition;
            if (!free(row[i], col[i]))
                throw new IllegalArgumentException("Drone start is not a free cell: D" + (i + 1));
            for (int j = 0; j < i; j++)
                if (row[j] == row[i] && col[j] == col[i])
                    throw new IllegalArgumentException("Drone initial cells must be distinct");
            d.path = new ArrayList<>();
            d.path.add(new Point(row[i], col[i]));
            d.frameIndex = 0;
            routes[i] = new ArrayList<>();
            visit(i, row[i], col[i]);
        }
        reachableFree = markReachable();
        simulate();
        coverage = reachableFree == 0 ? 0 : 100.0 * uniqueVisits / reachableFree;
        revisit = totalVisits == 0 ? 0 : 100.0 * (totalVisits - uniqueVisits) / totalVisits;
        int overlapCells = 0;
        for (BitSet b : visitors) if (b != null && b.cardinality() > 1) overlapCells++;
        overlap = uniqueVisits == 0 ? 0 : 100.0 * overlapCells / uniqueVisits;
        System.out.printf(Locale.ROOT,
                "Legacy A* benchmark: %s; steps=%d observed=%.2f%% visited=%.2f%% revisit=%.2f%% overlap=%.2f%%%n",
                terminationReason, steps, getObservedRatio(), coverage, revisit, overlap);
    }

    private boolean inside(int r, int c) {
        return r >= 0 && r < rows && c >= 0 && c < cols;
    }

    private boolean free(int r, int c) {
        return inside(r, c) && truth[r][c] != -1;
    }

    private boolean legal(int r, int c, int nr, int nc) {
        if (!free(nr, nc)) return false;
        int dr = nr - r, dc = nc - c;
        return dr == 0 || dc == 0 || (free(r, nc) && free(nr, c));
    }

    private int markReachable() {
        ArrayDeque<Point> q = new ArrayDeque<>();
        for (int i = 0; i < row.length; i++)
            if (!reachable[row[i]][col[i]]) {
                reachable[row[i]][col[i]] = true;
                q.add(new Point(row[i], col[i]));
            }
        int count = 0;
        while (!q.isEmpty()) {
            Point p = q.poll();
            count++;
            int r = p.getRow(), c = p.getCol();
            for (int[] d : N8) {
                int nr = r + d[0], nc = c + d[1];
                if (legal(r, c, nr, nc) && !reachable[nr][nc]) {
                    reachable[nr][nc] = true;
                    q.add(new Point(nr, nc));
                }
            }
        }
        return count;
    }

    private void visit(int id, int r, int c) {
        totalVisits++;
        if (visits[r][c]++ == 0) uniqueVisits++;
        int k = r * cols + c;
        if (visitors[k] == null) visitors[k] = new BitSet(drones.size());
        visitors[k].set(id);
    }

    private void discover(int r, int c, int t) {
        if (!inside(r, c)) return;
        if (map[r][c] == 0) {
            map[r][c] = truth[r][c] == -1 ? -1 : 1;
            if (map[r][c] == 1 && reachable[r][c]) {
                firstObservedAt[r][c] = t;
                observedReachable++;
            }
        }
    }

    /**
     * Sensor observation: 360 radial rays, obstacles block the field of view.
     */
    private void sense(int r, int c, int radius, int t) {
        discover(r, c, t);
        for (int degree = 0; degree < 360; degree++) {
            double theta = Math.toRadians(degree), dr = Math.sin(theta), dc = Math.cos(theta);
            double x = c + 0.5, y = r + 0.5;
            int prev = -1;
            for (double distance = 0.125; distance <= radius + 0.5; distance += 0.125) {
                int rr = (int) Math.floor(y + dr * distance), cc = (int) Math.floor(x + dc * distance);
                if (!inside(rr, cc)) break;
                int key = rr * cols + cc;
                if (key == prev) continue;
                prev = key;
                discover(rr, cc, t);
                if (truth[rr][cc] == -1) break;
            }
        }
    }

    /**
     * Old simulation's frontier uniqueness: a frontier belongs to the first
     * agent that detects it, until it is explored. Unlike a claim lease, this
     * is globally coordinated ownership, matching the legacy reference.
     */
    private void detectOwnedFrontiers() {
        // A known-free cell touching unknown is a standard reachable frontier.
        for (int i = 0; i < drones.size(); i++) {
            int radius = drones.get(i).sensorFoV;
            for (int r = Math.max(0, row[i] - radius); r <= Math.min(rows - 1, row[i] + radius); r++)
                for (int c = Math.max(0, col[i] - radius); c <= Math.min(cols - 1, col[i] + radius); c++) {
                    if (owner[r][c] != -1 || map[r][c] != 1) continue;
                    if (Math.hypot(r - row[i], c - col[i]) > radius + 0.5) continue;
                    if (isFrontier(r, c)) owner[r][c] = i;
                }
        }
        // Once it is no longer a frontier, allow another frontier to be assigned.
        for (int r = 0; r < rows; r++)
            for (int c = 0; c < cols; c++) if (owner[r][c] >= 0 && !isFrontier(r, c)) owner[r][c] = -1;
    }

    private boolean isFrontier(int r, int c) {
        if (map[r][c] != 1) return false;
        for (int[] v : N8) {
            int nr = r + v[0], nc = c + v[1];
            if (inside(nr, nc) && map[nr][nc] == 0) return true;
        }
        return false;
    }

    /**
     * A* to nearest owned frontier, using geometric distances, not hop count.
     * The heuristic is the minimum octile distance to any owned candidate,
     * so the first popped frontier is the globally shortest reachable one.
     */
    private List<Point> astarNearest(int id) {
        boolean[][] goalsMask = new boolean[rows][cols];
        ArrayList<Point> targets = new ArrayList<>();
        for (int r = 0; r < rows; r++)
            for (int c = 0; c < cols; c++)
                if (owner[r][c] == id && isFrontier(r, c)) {
                    goalsMask[r][c] = true;
                    targets.add(new Point(r, c));
                }
        if (targets.isEmpty()) return Collections.emptyList();
        double[][] best = new double[rows][cols];
        for (double[] a : best) Arrays.fill(a, Double.POSITIVE_INFINITY);
        PriorityQueue<Node> q = new PriorityQueue<>(Comparator.comparingDouble((Node n) -> n.f).thenComparingDouble(n -> n.g).thenComparingInt(n -> n.r).thenComparingInt(n -> n.c));
        int sr = row[id], sc = col[id];
        best[sr][sc] = 0;
        q.add(new Node(sr, sc, 0, heuristic(sr, sc, targets), null));
        while (!q.isEmpty()) {
            Node p = q.poll();
            if (p.g > best[p.r][p.c] + 1e-9) continue;
            if (goalsMask[p.r][p.c]) {
                LinkedList<Point> result = new LinkedList<>();
                for (Node n = p; n != null; n = n.parent) result.addFirst(new Point(n.r, n.c));
                return result;
            }
            for (int[] v : N8) {
                int nr = p.r + v[0], nc = p.c + v[1];
                // LEGACY ORACLE: uses true environment for safe paths, just as
                // old base_map. This must not be reported as local-map planning.
                if (!legal(p.r, p.c, nr, nc)) continue;
                double ng = p.g + (v[0] == 0 || v[1] == 0 ? 1 : SQRT2);
                if (ng + 1e-9 < best[nr][nc]) {
                    best[nr][nc] = ng;
                    q.add(new Node(nr, nc, ng, ng + heuristic(nr, nc, targets), p));
                }
            }
        }
        return Collections.emptyList();
    }

    private double heuristic(int r, int c, List<Point> targets) {
        double h = Double.POSITIVE_INFINITY;
        for (Point p : targets) {
            int dr = Math.abs(r - p.getRow()), dc = Math.abs(c - p.getCol());
            h = Math.min(h, Math.max(dr, dc) + (SQRT2 - 1) * Math.min(dr, dc));
        }
        return h;
    }

    private void simulate() {
        if (drones.isEmpty()) {
            terminationReason = "NO_DRONES";
            return;
        }
        int stagnant = 0;
        for (int t = 0; t < MAX_STEPS; t++) {
            for (int i = 0; i < drones.size(); i++) sense(row[i], col[i], drones.get(i).sensorFoV, t);
            if (observedReachable == reachableFree) {
                terminationReason = "ALL_REACHABLE_FREE_CELLS_OBSERVED";
                steps = t;
                break;
            }
            detectOwnedFrontiers();
            int n = drones.size();
            Point[] next = new Point[n];
            for (int i = 0; i < n; i++) {
                Point goal = goals[i];
                boolean valid = goal != null && owner[goal.getRow()][goal.getCol()] == i && isFrontier(goal.getRow(), goal.getCol());
                if (!valid || routes[i].size() < 2) {
                    List<Point> fresh = astarNearest(i);
                    routes[i] = fresh;
                    goals[i] = fresh.isEmpty() ? null : fresh.get(fresh.size() - 1);
                }
                // If the next waypoint was invalidated, replan again.
                if (routes[i].size() > 1) {
                    Point step = routes[i].get(1);
                    next[i] = legal(row[i], col[i], step.getRow(), step.getCol()) ? step : new Point(row[i], col[i]);
                } else next[i] = new Point(row[i], col[i]);
            }
            // Simultaneous collision filtering, lowest ID wins destination tie.
            for (int i = 0; i < n; i++)
                for (int j = i + 1; j < n; j++) {
                    if (next[i].getRow() == next[j].getRow() && next[i].getCol() == next[j].getCol())
                        next[j] = new Point(row[j], col[j]);
                }
            for (int k = 0; k < n * n; k++) {
                boolean changed = false;
                for (int i = 0; i < n; i++)
                    for (int j = 0; j < n; j++)
                        if (i != j) {
                            if (next[i].getRow() == row[j] && next[i].getCol() == col[j] && next[j].getRow() == row[j] && next[j].getCol() == col[j]) {
                                next[i] = new Point(row[i], col[i]);
                                changed = true;
                            }
                            if (i < j && next[i].getRow() == row[j] && next[i].getCol() == col[j] && next[j].getRow() == row[i] && next[j].getCol() == col[i]) {
                                next[i] = new Point(row[i], col[i]);
                                next[j] = new Point(row[j], col[j]);
                                changed = true;
                            }
                            // Also reject opposite diagonal segments crossing at cell corners.
                            if (i < j && row[i] + row[j] == next[i].getRow() + next[j].getRow()
                                    && col[i] + col[j] == next[i].getCol() + next[j].getCol()
                                    && (row[i] != next[i].getRow() || col[i] != next[i].getCol())
                                    && (row[j] != next[j].getRow() || col[j] != next[j].getCol())
                                    && !(row[i] == row[j] && col[i] == col[j])
                                    && (row[i] + next[i].getRow() == row[j] + next[j].getRow())
                                    && (col[i] + next[i].getCol() == col[j] + next[j].getCol())) {
                                next[j] = new Point(row[j], col[j]);
                                changed = true;
                            }
                        }
                if (!changed) break;
            }
            boolean moved = false;
            for (int i = 0; i < n; i++) {
                int nr = next[i].getRow(), nc = next[i].getCol();
                if (nr != row[i] || nc != col[i]) {
                    row[i] = nr;
                    col[i] = nc;
                    visit(i, nr, nc);
                    moved = true;
                    if (routes[i].size() > 1 && routes[i].get(1).getRow() == nr && routes[i].get(1).getCol() == nc)
                        routes[i].remove(0);
                } else if (routes[i].size() > 1) routes[i] = Collections.emptyList();
                drones.get(i).path.add(new Point(row[i], col[i]));
            }
            steps = t + 1;
            stagnant = moved ? 0 : stagnant + 1;
            if (stagnant >= 30) {
                terminationReason = "STALL_30_STEPS";
                break;
            }
        }
        for (int i = 0; i < drones.size(); i++) sense(row[i], col[i], drones.get(i).sensorFoV, steps);
        if (observedReachable == reachableFree) terminationReason = "ALL_REACHABLE_FREE_CELLS_OBSERVED";
        if (terminationReason.equals("NOT_STARTED")) terminationReason = "MAX_STEPS";
        for (Drone d : drones) {
            d.frameIndex = 0;
        }
    }

    public int[][] getFirstObservedAt() {
        return firstObservedAt;
    }

    public boolean[][] getMustExploreMask() {
        return reachable;
    }

    public int getSteps() {
        return steps;
    }

    public double getCoverage() {
        return coverage;
    }

    public double getRevisit() {
        return revisit;
    }

    public double getOverlap() {
        return overlap;
    }

    public double getObservedRatio() {
        return reachableFree == 0 ? 100 : 100.0 * observedReachable / reachableFree;
    }

    public String getTerminationReason() {
        return terminationReason;
    }

    private record Node(int r, int c, double g, double f, Node parent) {
    }
}
