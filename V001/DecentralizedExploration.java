package swarmdrone.decentralizedswarmcoverage;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

public class DecentralizedExploration {

    private final int[][] neighbors = {// DRONES
            {0, 1},/*east*/
            {1, 1},/*southeast*/
            {1, 0},/*south*/
            {1, -1},/*southwest*/
            {0, -1},/*west*/
            {-1, -1},/*northwest*/
            {-1, 0},/*north*/
            {-1, 1},/*northeast*/
    };// DRONES
    private final ArrayList<Drone> drones;
    private final int MIN_ROW;// EXPLORATION
    private final int MIN_COL;// EXPLORATION
    private final int MAX_ROW;// EXPLORATION
    private final int MAX_COL;// EXPLORATION
    protected int[][] environment;
    protected int[][] mustBeExploredArea;// Unknown area that must be explored
    protected int mustBeExplored = 0;// number of cell that must be explored
    protected double[][] pheromoneGrid;
    // pheromone
    /*
    | Parameter     | Range       |
    | ------------- | ----------- |
    | Q (deposit)   | 0.5  – 2    |
    | ρ (decay)     | 0.03 – 0.08 |
    | D (diffusion) | 0.05 – 0.15 |
    | P_max         | 5    – 20   |
    */
    protected double rho = 0.05;
    protected double D = 0.1;
    private int numRows = 0;// GRID
    private int numCols = 0;// GRID
    private int[][] map;// obstale = -1; unknown/unexplored = 0 ; explored/known/free = value>=1// map ini sudah diperkecil dari environment hanya fokus ke area komputasi

    public DecentralizedExploration(int[][] environment, ArrayList<Drone> drones, int MIN_ROW, int MIN_COL, int MAX_ROW, int MAX_COL) {
        this.environment = environment;
        this.drones = drones;
        this.MIN_ROW = MIN_ROW;
        this.MIN_COL = MIN_COL;
        this.MAX_ROW = MAX_ROW;
        this.MAX_COL = MAX_COL;
        run();
    }

    private void initializeMap() {
        numRows = environment.length;
        numCols = environment[0].length;
        // initialize map by copy value from environment
        map = new int[numRows][numCols];
        for (int i = 0; i < map.length; i++) {
            System.arraycopy(environment[i], 0, map[i], 0, map[i].length);
        }
        // initialize unknown area
        mustBeExploredArea = getUnknownArea();
        // initialize pheromone
        pheromoneGrid = new double[numRows][numCols];
        for (int i = 0; i < map.length; i++) {
            for (int j = 0; j < map[i].length; j++) {
                if (map[i][j] == 0 && mustBeExploredArea[i][j] == 1) {
                    pheromoneGrid[i][j] = 1;
                }
            }
        }
    }

    protected int[][] getUnknownArea() {
        int[][] unknownArea = new int[numRows][numCols];
        if (drones != null && !drones.isEmpty()) {
            for (int i = 0; i < drones.size(); i++) {
                Queue<Point> queue = new LinkedList<>();
                Point point = new Point(drones.get(i).initialRowPosition, drones.get(i).initialColPosition);
                if (environment[point.getRow()][point.getCol()] == 0) {
                    queue.offer(point);
                    unknownArea[point.getRow()][point.getCol()] = 1;
                }
                while (!queue.isEmpty()) {
                    Point center = queue.poll();
                    for (int n = 0; n < neighbors.length; n++) {
                        int row = center.getRow() + neighbors[n][0];
                        int col = center.getCol() + neighbors[n][1];
                        if (row >= MIN_ROW && row <= MAX_ROW && col >= MIN_COL && col <= MAX_COL && unknownArea[row][col] == 0 && environment[row][col] == 0) {
                            unknownArea[row][col] = 1;
                            queue.offer(new Point(row, col));
                        }
                    }
                }
            }
        }
        return unknownArea;
    }

    protected int getMustBeExplored(int[][] unknownArea) {
        mustBeExplored = 0;
        if (unknownArea != null) {
            for (int i = 0; i < unknownArea.length; i++) {
                for (int j = 0; j < unknownArea[i].length; j++) {
                    if (unknownArea[i][j] == 1) {
                        mustBeExplored++;
                    }
                }
            }
        }
        return mustBeExplored;
    }

    protected void run() {
        if (environment != null && drones != null && !drones.isEmpty()) {
            initializeMap();
            double[][] deltaTau = new double[numRows][numCols];
            //initialize drone
            for (int i = 0; i < drones.size(); i++) {
                drones.get(i).setNeighbors(neighbors);
                drones.get(i).setMap(map);
                drones.get(i).setMustBeExploredArea(mustBeExploredArea);
                drones.get(i).setPheromoneGrid(pheromoneGrid);
                drones.get(i).initializePosition();
                drones.get(i).setDeltaTau(deltaTau);
                drones.get(i).setFree(drones.get(i).initialRowPosition, drones.get(i).initialColPosition);
            }
            // SEARCHING
            boolean isThereStillAFrontier = true;
            while (isThereStillAFrontier) {
                isThereStillAFrontier = false;
                // EVERY DRONE DO PATH PLANNING
                for (int i = 0; i < drones.size(); i++) {
                    drones.get(i).detectFrontiers();
                    drones.get(i).frontierValidation();
                    if (drones.get(i).isThereStillAFrontier()) {
                        isThereStillAFrontier = true;
                    }
                    drones.get(i).pathPlanningToTheNearestFrontier();
                }
                // PHEROMONE DEPOSIT
                for (int i = 0; i < pheromoneGrid.length; i++) {
                    for (int j = 0; j < pheromoneGrid[i].length; j++) {
                        if (mustBeExploredArea[i][j] == 1) {
                            double tau = (1.0 - rho) * pheromoneGrid[i][j] + deltaTau[i][j];
                            pheromoneGrid[i][j] += tau;
                        }
                    }
                }
                // DIFFUSION
                double[][] newPheromoneGrid = new double[numRows][numCols];
                for (int i = 0; i < pheromoneGrid.length; i++) {
                    for (int j = 0; j < pheromoneGrid[i].length; j++) {
                        if (mustBeExploredArea[i][j] == 1) {
                            double center = pheromoneGrid[i][j];
                            // difusion
                            newPheromoneGrid[i][j] = (1.0 - D) * center;
                            double sum = 0;
                            double n = 0;
                            for (int k = 0; k < neighbors.length; k++) {
                                int row = i + neighbors[k][0];
                                int col = j + neighbors[k][1];
                                if (row >= 0 && row < numRows && col >= 0 && col < numCols && environment[row][col] >= 0 && pheromoneGrid[row][col] > 0 && mustBeExploredArea[row][col] == 1) {
                                    n++;
                                    sum += pheromoneGrid[row][col];
                                }
                            }
                            if (sum > 0 && n > 0) {
                                double avg = sum / n;
                                // difusion
                                newPheromoneGrid[i][j] = newPheromoneGrid[i][j] + D * avg;
                            }
                        }
                    }
                }
                // UPDATE PHEROMONE
                for (int i = 0; i < pheromoneGrid.length; i++) {
                    System.arraycopy(newPheromoneGrid[i], 0, pheromoneGrid[i], 0, pheromoneGrid[i].length);
                }
                //reset delta tau
                deltaTau = new double[numRows][numCols];
                for (int i = 0; i < drones.size(); i++) {
                    drones.get(i).setDeltaTau(deltaTau);
                }
            }// sudah tidak ada frontier pencarian selesai

            //cetak hasil
            for (int i = 0; i < drones.size(); i++) {
                System.out.println("path: " + drones.get(i).path);
            }

        }
    }

}