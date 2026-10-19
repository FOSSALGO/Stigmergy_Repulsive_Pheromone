package swarmdrone.decentralizedswarmcoverage;

import java.util.*;

/**
 * Simulation scheduler and environmental stigmergy. NOT a swarm task allocator.
 * Each Drone owns its private map, FTFD, utility decision, and A* planner.
 */
public class DecentralizedExploration {
    // Preliminary experimental parameters, not scientifically tuned optima.
    static final double Q = 1.0, RHO = 0.05, PHEROMONE_MAX = 10.0, LAMBDA = 0.5;
    static final double DISTANCE_REFERENCE = 50.0;
    static final int MAX_STEPS = 1200;
    private final int[][] environment;
    private final ArrayList<Drone> drones;
    private final double[][] pheromone;
    private final int[][] visits;
    private boolean[][] mustExploreMask;
    private final int[][] firstObservedAt; // global union FOR REPLAY ONLY, never read by agents
    private final BitSet[] visitors;
    private final int rows, cols;
    private int steps, uniqueVisits, totalVisits, reachableFree;
    private double coverage, revisit, overlap;

    /** Compatibility with CanvasView.run(): results are computed, then paths replay in renderer. */
    public DecentralizedExploration(int[][] environment, ArrayList<Drone> drones,
                                   int minRow, int minCol, int maxRow, int maxCol) {
        this.environment=environment;
        this.drones=drones;
        rows=environment.length;cols=environment[0].length;
        pheromone=new double[rows][cols];
        visits=new int[rows][cols];
        firstObservedAt=new int[rows][cols];
        for (int[] line : firstObservedAt) Arrays.fill(line, -1);
        visitors=new BitSet[rows*cols];
        if (drones.isEmpty()) return;
        // Never copy ground-truth occupancy into the agents.
        validateStarts();
        for (int i=0;i<drones.size();i++)
            drones.get(i).initializeAgent(i+1,RHO,PHEROMONE_MAX,LAMBDA,DISTANCE_REFERENCE);
        reachableFree=countReachableFree();
        for (int i=0;i<drones.size();i++) {
            Drone d=drones.get(i);
            registerVisit(d.current(),i);
            Point p=d.current();
            pheromone[p.getRow()][p.getCol()]=Math.min(PHEROMONE_MAX,pheromone[p.getRow()][p.getCol()]+Q);
        }
        simulate();
        for (Drone d:drones) { d.visualization="PAUSE";d.frameIndex=0; }
        System.out.printf("Exploration complete: steps=%d coverage=%.2f%% revisit=%.2f%% overlap=%.2f%%%n",
            steps,coverage,revisit,overlap);
    }

    private void validateStarts() {
        Set<Integer> seen=new HashSet<>();
        for (Drone d:drones) {
            int r=d.initialRowPosition,c=d.initialColPosition;
            if(r<0||r>=rows||c<0||c>=cols||environment[r][c]==-1)
                throw new IllegalArgumentException("Drone start must be in a free environment cell");
            if(!seen.add(r*cols+c))
                throw new IllegalArgumentException("Drone initial positions must be distinct");
        }
    }

    private int countReachableFree() {
        boolean[][] seen=new boolean[rows][cols];
        ArrayDeque<Point> q=new ArrayDeque<>();
        for(Drone d:drones) {
            int r=d.initialRowPosition,c=d.initialColPosition;
            seen[r][c]=true;q.add(new Point(r,c));
        }
        int count=0;
        while(!q.isEmpty()) {
            Point p=q.removeFirst();count++;
            for(int[] v:Drone.N8) {
                int nr=p.getRow()+v[0],nc=p.getCol()+v[1];
                if(nr<0||nr>=rows||nc<0||nc>=cols||seen[nr][nc]||environment[nr][nc]==-1)continue;
                if(v[0]!=0&&v[1]!=0 && (environment[p.getRow()][nc]==-1||environment[nr][p.getCol()]==-1))continue;
                seen[nr][nc]=true;q.add(new Point(nr,nc));
            }
        }
        mustExploreMask=seen;
        return count;
    }

    private void simulate() {
        int stall=0;
        for (int t=0;t<MAX_STEPS;t++) {
            // Snapshot observation: all drones receive the same time-indexed field.
            for (Drone d:drones) d.observe(environment,pheromone,t);
            captureObserved(t);
            for (Drone d:drones) d.updateFTFD();
            Point[] starts=new Point[drones.size()];
            Point[] proposed=new Point[drones.size()];
            for(int i=0;i<drones.size();i++) {
                Drone d=drones.get(i);
                starts[i]=d.current();
                proposed[i]=d.propose(t);
            }
            Point[] approved=resolveMoves(starts,proposed);
            double[][] deposits=new double[rows][cols];
            boolean moved=false;
            for(int i=0;i<drones.size();i++) {
                Drone d=drones.get(i);
                if(d.move(approved[i])) {
                    moved=true;
                    Point p=d.current();
                    deposits[p.getRow()][p.getCol()]+=Q;
                    registerVisit(p,i);
                }
            }
            for(int r=0;r<rows;r++) for(int c=0;c<cols;c++) {
                if(environment[r][c]==-1)continue;
                pheromone[r][c]=Math.min(PHEROMONE_MAX,(1-RHO)*pheromone[r][c]+deposits[r][c]);
            }
            for (Drone d : drones) d.recordFrame(); // includes waiting drones
            steps=t+1;
            if(moved)stall=0;else stall++;
            // A bounded horizon is the primary experiment termination rule.
            // A prolonged stationary configuration is explicitly reported as stalled.
            if(stall>=30)break;
        }
        // A last observation at the final positions closes the replay timeline.
        for (Drone d : drones) d.observe(environment,pheromone,steps);
        captureObserved(steps);
        int multiple=0;
        for(BitSet visitor:visitors) if(visitor!=null && visitor.cardinality()>=2)multiple++;
        coverage=reachableFree==0?0:100.0*uniqueVisits/reachableFree;
        revisit=totalVisits==0?0:100.0*(totalVisits-uniqueVisits)/totalVisits;
        overlap=uniqueVisits==0?0:100.0*multiple/uniqueVisits;
    }

    /** Union of locally OBSERVED cells, kept only for dark-to-light playback. */
    private void captureObserved(int frame) {
        for (Drone d : drones) {
            for (int k : d.changedObservationKeys()) {
                int r=k/cols,c=k%cols;
                if (firstObservedAt[r][c] < 0 && d.knownCell(r,c) != Drone.UNKNOWN)
                    firstObservedAt[r][c] = frame;
            }
        }
    }

    private void registerVisit(Point p,int idIndex) {
        int r=p.getRow(),c=p.getCol(),key=r*cols+c;
        if(visits[r][c]++==0)uniqueVisits++;
        totalVisits++;
        if(visitors[key]==null)visitors[key]=new BitSet(drones.size());
        visitors[key].set(idIndex);
    }

    /** Atomic collision arbitration: the smallest ID wins a contested destination. */
    private Point[] resolveMoves(Point[] current,Point[] desired) {
        int n=current.length;
        Point[] approved=desired.clone();
        boolean change;
        do {
            change=false;
            // Contested destinations: sorted agent index matches ascending ID.
            Map<Integer,Integer> winner=new HashMap<>();
            for(int i=0;i<n;i++) {
                if(equal(current[i],approved[i]))continue;
                int key=approved[i].getRow()*cols+approved[i].getCol();
                winner.putIfAbsent(key,i);
            }
            for(int i=0;i<n;i++) {
                if(equal(current[i],approved[i]))continue;
                int key=approved[i].getRow()*cols+approved[i].getCol();
                if(winner.get(key)!=i){approved[i]=current[i];change=true;}
            }
            // Occupied by an agent that will not leave: all incoming proposals lose.
            for(int i=0;i<n;i++) {
                if(equal(current[i],approved[i]))continue;
                for(int j=0;j<n;j++) if(i!=j && equal(approved[i],current[j]) && equal(current[j],approved[j])) {
                    approved[i]=current[i];change=true;break;
                }
            }
            // Swap and crossing diagonals are forbidden. Smaller ID retains priority on crossings.
            for(int i=0;i<n;i++)for(int j=i+1;j<n;j++) {
                if(equal(current[i],approved[i])||equal(current[j],approved[j]))continue;
                boolean swap=equal(approved[i],current[j])&&equal(approved[j],current[i]);
                if(swap){approved[i]=current[i];approved[j]=current[j];change=true;continue;}
                if(intersectsInterior(current[i],approved[i],current[j],approved[j])) {
                    approved[j]=current[j];change=true;
                }
            }
        }while(change);
        return approved;
    }

    private static boolean equal(Point a,Point b) {
        return a.getRow()==b.getRow()&&a.getCol()==b.getCol();
    }
    /** Strict crossing of two movement segments in the same timestep. */
    private static boolean intersectsInterior(Point a,Point b,Point c,Point d) {
        double ax=a.getCol(),ay=a.getRow(),bx=b.getCol(),by=b.getRow();
        double cx=c.getCol(),cy=c.getRow(),dx=d.getCol(),dy=d.getRow();
        double u=(bx-ax)*(cy-ay)-(by-ay)*(cx-ax);
        double v=(bx-ax)*(dy-ay)-(by-ay)*(dx-ax);
        double w=(dx-cx)*(ay-cy)-(dy-cy)*(ax-cx);
        double z=(dx-cx)*(by-cy)-(dy-cy)*(bx-cx);
        return u*v<0&&w*z<0;
    }

    public int[][] getFirstObservedAt() { return firstObservedAt; }
    public boolean[][] getMustExploreMask() { return mustExploreMask; }
    public int getSteps(){return steps;}
    public double getCoverage(){return coverage;}
    public double getRevisit(){return revisit;}
    public double getOverlap(){return overlap;}
    public double[][] getPheromone(){return pheromone;}
    public int[][] getVisits(){return visits;}
}
