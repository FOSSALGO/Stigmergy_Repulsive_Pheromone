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
    private final int neighbourhood = 1;

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

    void initializeAgent(int id, double rho, double pMax, double pReference, double lambda, double distanceReference) {
        this.id = id;
        this.name = "D" + id;
        this.rho = rho;
        this.pMax = pMax;
        this.pReference = pReference;
        this.lambda = lambda;
        this.distanceReference = distanceReference;
        int rows = environment.length, cols = environment[0].length;
        localMap = new int[rows][cols];
        localPheromone = new double[rows][cols];
        observedAt = new int[rows][cols];
        for (int[] line : observedAt) java.util.Arrays.fill(line, -1);
        frontiers.clear();
        changed.clear();
        route.clear();
        target = null;
        waitSteps = 0;
        lastSelectionAt = -1;
        targetSwitches = 0;
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
                }
                localPheromone[r][c] = pheromone[r][c];
                observedAt[r][c] = timestep;
                if (status == OCCUPIED) break;
            }
        }
        localMap[rowPosition][colPosition] = FREE;
        changed.add(key(rowPosition, colPosition));
    }

    /**
     * Incremental frontier-tracing update: revisit the changed boundary, never scan the entire map.
     */
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

    private double frontierPheromone(Point f, int t) {
        double sum = 0;
        int count = 0;
        for (int dr = -neighbourhood; dr <= neighbourhood; dr++)
            for (int dc = -neighbourhood; dc <= neighbourhood; dc++) {
                if (dr * dr + dc * dc > neighbourhood * neighbourhood) continue;
                int r = f.getRow() + dr, c = f.getCol() + dc;
                if (!inside(r, c) || localMap[r][c] == OCCUPIED) continue;
                sum += pheromoneAt(r, c, t);
                count++;
            }
        return count > 0 ? sum / count : 0;
    }

    /**
     * Reconsider a still-valid target periodically when new LOCAL pheromone
     * information is available. Hysteresis prevents frequent target oscillation.
     * No drone reads another drone's frontier, route or occupancy map.
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
        boolean review = target == null || (timestep - lastSelectionAt >= RECHECK_INTERVAL);
        if (review) {

            double bestScore = Double.NEGATIVE_INFINITY;
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
                double pheromone = Math.min(1.0, frontierPheromone(f, timestep) / pReference);
                double score = gain - distance - lambda * pheromone;
                if (same(target, f)) {
                    currentScore = score;
                    currentPath = candidate;
                }
                if (score > bestScore + EPS ||
                        (Math.abs(score - bestScore) <= EPS &&
                                bestTarget != null && routeDistance(candidate) < routeDistance(bestPath) - EPS)) {
                    bestScore = score;
                    bestTarget = f;
                    bestPath = candidate;
                }
            }

            if (bestTarget != null) {
                // A still-valid target is retained unless a distinctly better
                // option appears; pheromone changes can now change a committed goal.
                if (target == null || currentPath.isEmpty() ||
                        bestScore > currentScore + SWITCH_MARGIN) {
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
