package swarmdrone.decentralizedswarmcoverage_ijrcs;

import javafx.scene.canvas.GraphicsContext;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import java.util.ArrayList;

/**
 * Visual representation and replay data for a swarm drone.
 * All exploration decisions are made by DecentralizedExploration.
 */
public class Drone {
    // Never change these colors without intentionally revising the UI.
    private static final String[] PATH_COLORS = {
            "#FF204E", "#00C2FF", "#FFB000", "#7D3CFF", "#00C878",
            "#FF5E00", "#EC00B9", "#0075FF", "#A4D600", "#FF4466",
            "#00A9A5", "#FF8C00", "#9D00FF", "#20BF55", "#FF006E",
            "#00B4D8", "#D95F02", "#8F00FF", "#5BA000", "#E63946",
            "#4361EE", "#F77F00", "#7209B7", "#00B894", "#FB5607",
            "#3A86FF", "#E500A4", "#6A994E", "#F94144", "#00A6FB"
    };

    protected final int[][] environment; // Renderer-only obstacle occlusion.
    protected int initialRowPosition, initialColPosition;
    protected int rowPosition, colPosition; // Initial fallback before replay frames exist.
    protected int sensorFoV, cellSize;
    protected String name = "";
    protected ArrayList<Point> path = new ArrayList<>();
    protected int frameIndex = -1;
    private String pathColor = PATH_COLORS[0];

    public Drone(int[][] environment, int row, int col, int sensorFoV, int cellSize) {
        this.environment = environment;
        this.initialRowPosition = row;
        this.initialColPosition = col;
        this.rowPosition = row;
        this.colPosition = col;
        this.sensorFoV = sensorFoV;
        this.cellSize = cellSize;
        path.add(new Point(row, col));
    }

    protected void setName(String name) { this.name = name; }
    static int pathPaletteSize() { return PATH_COLORS.length; }
    void setPathColorByIndex(int index) {
        pathColor = PATH_COLORS[Math.floorMod(index, PATH_COLORS.length)];
    }
    String getPathColor() { return pathColor; }

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

}
