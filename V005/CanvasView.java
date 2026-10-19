package swarmdrone.decentralizedswarmcoverage;

import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.ScrollEvent;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.FontSmoothingType;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;

public class CanvasView extends Canvas {

    private static final double MIN_SCALE = 0.1;
    private static final double MAX_SCALE = 20.0;
    private static final double ZOOM_FACTOR = 1.05;

    /**
     * Ambang perpindahan pointer (dalam pixel screen) yang dianggap sebagai drag.
     * Gerakan kecil di bawah nilai ini tetap diperlakukan sebagai klik biasa.
     */
    private static final double DRAG_THRESHOLD_PX = 4.0;
    private final String delimiter = ":"; // FILE
    private final int sensorFoV = 4; // DRONES, satuan: cell
    private final int cameraFoV = 3; // DRONES, satuan: cell
    private final int MAX_NUM_ROWS = 200; // GRID
    private final int MAX_NUM_COLS = 400; // GRID
    private final int[][] map = null;
    private final int[][] base_map = null;
    private final double[][] pheromoneGrid = null;
    protected double translateX = 0.0;
    protected double translateY = 0.0;

    protected double scale = 1.0;
    private double mouseAnchorX = 0.0;
    private double mouseAnchorY = 0.0;

    // =========================================================
    // POINTER GESTURE STATE
    // =========================================================
    private double mousePressX = 0.0;
    private double mousePressY = 0.0;
    private boolean mousePressActive = false;
    private boolean dragGestureDetected = false;
    private boolean zoomGestureDetected = false;
    private boolean suppressNextClick = false;
    private boolean panningCamera = false;
    private MouseButton pressedMouseButton = MouseButton.NONE;
    private ArrayList<Drone> drones = new ArrayList<>();
    private int draggingDrone = -1;
    private int moveToI = -1;
    private int moveToJ = -1;
    private int lineToI = -1;
    private int lineToJ = -1;
    private int positionI = -1;
    private int positionJ = -1;
    private int cellSize;
    private double halfCellSize;
    private int numRows;


    // =========================================================
    // EXPLORATION
    // =========================================================
    private int numCols;
    private int[][] environment = null;
    private String[][] wallType = new String[0][0];
    private int MIN_ROW;
    private int MIN_COL;
    private int MAX_ROW;
    private int MAX_COL;

    private State state = State.DEFAULT;

    // Playback is synchronized across all agents, including waiting steps.
    private DecentralizedExploration lastExploration;
    private int playbackFrame = 0;
    private boolean playbackPlaying = false;
    private long lastPlaybackTick = 0;
    private static final long PLAYBACK_INTERVAL_NS = 95_000_000L; // ~10.5 timesteps/sec

    protected boolean hasReplay() { return lastExploration != null; }
    protected int playbackFrame() { return playbackFrame; }
    protected int totalPlaybackFrames() { return lastExploration == null ? 0 : lastExploration.getSteps(); }
    protected boolean isPlaybackPlaying() { return playbackPlaying; }

    private void setPlaybackFrame(int frame) {
        if (lastExploration == null) return;
        playbackFrame = Math.max(0, Math.min(frame, lastExploration.getSteps()));
        for (Drone d : drones) {
            d.frameIndex = Math.min(playbackFrame, d.path.size() - 1);
            d.visualization = "PAUSE"; // playback controlled by CanvasView clock
        }
    }

    protected void togglePlayPause() {
        if (!hasReplay()) return;
        if (playbackFrame >= totalPlaybackFrames()) setPlaybackFrame(0);
        playbackPlaying = !playbackPlaying;
        lastPlaybackTick = System.nanoTime();
        render();
    }

    protected void replay() {
        if (!hasReplay()) return;
        setPlaybackFrame(0);
        playbackPlaying = true;
        lastPlaybackTick = System.nanoTime();
        render();
    }

    private void clearReplay() {
        lastExploration = null;
        playbackPlaying = false;
        playbackFrame = 0;
        lastPlaybackTick = 0;
    }



    protected CanvasView(int[][] environment, int cellSize) {
        super();
        if (environment != null && environment.length > 0 && environment[0] != null && environment[0].length > 0 && cellSize > 0) {
            this.environment = environment;
            numRows = environment.length;
            numCols = environment[0].length;
            this.cellSize = cellSize;
            this.halfCellSize = 0.5 * cellSize;
            wallType = new String[numRows][numCols]; /* Ukuran Canvas merepresentasikan ukuran world/grid awal. Transformasi kamera dilakukan melalui translateX, translateY, dan scale. */
            setWidth(numCols * cellSize);
            setHeight(numRows * cellSize);
            checkWallType();
        }
    }

    // =========================================================
    // COORDINATE CONVERSION
    // =========================================================

    /**
     * Mengubah koordinat X SCREEN
     * menjadi koordinat X WORLD.
     * <p>
     * ScreenX = WorldX * scale + translateX
     * <p>
     * WorldX = (ScreenX - translateX) / scale
     */
    private double screenToWorldX(double screenX) {
        return (screenX - translateX) / scale;
    }


    /**
     * Mengubah koordinat Y SCREEN
     * menjadi koordinat Y WORLD.
     */
    private double screenToWorldY(double screenY) {
        return (screenY - translateY) / scale;
    }


    /**
     * Mengubah koordinat X WORLD
     * menjadi koordinat X SCREEN.
     */
    private double worldToScreenX(double worldX) {
        return worldX * scale + translateX;
    }


    /**
     * Mengubah koordinat Y WORLD
     * menjadi koordinat Y SCREEN.
     */
    private double worldToScreenY(double worldY) {
        return worldY * scale + translateY;
    }


    /**
     * Mengambil kolom grid dari posisi mouse.
     */
    private int getColumnFromMouse(double mouseX) {
        double worldX = screenToWorldX(mouseX);
        return (int) Math.floor(worldX / cellSize);
    }


    /**
     * Mengambil baris grid dari posisi mouse.
     */
    private int getRowFromMouse(double mouseY) {
        double worldY = screenToWorldY(mouseY);
        return (int) Math.floor(worldY / cellSize);
    }


    /**
     * Memeriksa apakah posisi mouse berada
     * di dalam area environment.
     */
    private boolean isMouseInsideEnvironment(double mouseX, double mouseY) {
        double worldX = screenToWorldX(mouseX), worldY = screenToWorldY(mouseY);
        return worldX >= 0 && worldX < numCols * cellSize && worldY >= 0 && worldY < numRows * cellSize;
    }


    /**
     * Memeriksa apakah indeks grid valid.
     */
    private boolean isValidGridPosition(int row, int col) {
        return row >= 0 && row < numRows && col >= 0 && col < numCols;
    }

    // =========================================================
    // FILE
    // =========================================================
    protected void writeToFile(File file) {
        if (environment != null) try {
            StringBuffer sb = new StringBuffer().append("ENVIRONMENT----------------------------------\n").append("NUM ROWS" + delimiter + environment.length + "\n").append("NUM COLS" + delimiter + environment[0].length + "\n");
            for (int i = 0; i < environment.length; i++) {
                for (int j = 0; j < environment[i].length; j++) sb.append(environment[i][j] + delimiter);
                sb.append("\n");
            }
            if (drones != null && !drones.isEmpty()) {
                sb.append("DRONES---------------------------------------\n").append("NUM DRONES" + delimiter + drones.size() + "\n");
                for (int d = 0; d < drones.size(); d++)
                    sb.append(drones.get(d).initialRowPosition + delimiter + drones.get(d).initialColPosition + "\n");
            }
            String data = sb.toString();
            Path filePath = file.toPath();
            Files.write(filePath, data.getBytes(), StandardOpenOption.CREATE, StandardOpenOption.TRUNCATE_EXISTING);
            state = State.DEFAULT;
        } catch (IOException e) {
            throw new java.io.UncheckedIOException("Could not save environment", e);
        }
    }


    /** Read both original .sc files and newly saved files, transactionally. */
    protected void readFromFile(File file) {
        try {
            java.util.List<String> lines = Files.readAllLines(file.toPath());
            int index = 0;
            if (lines.isEmpty()) throw new IOException("Empty environment file");
            if (lines.get(index).startsWith("ENVIRONMENT")) index++;
            int r = readHeaderInteger(lines, index++, "NUM ROWS");
            int c = readHeaderInteger(lines, index++, "NUM COLS");
            if (r <= 0 || c <= 0 || r > MAX_NUM_ROWS || c > MAX_NUM_COLS)
                throw new IOException("Invalid grid size: " + r + " x " + c);
            int[][] loaded = new int[r][c];
            for (int row=0; row<r; row++) {
                if (index >= lines.size()) throw new IOException("Missing grid row " + row);
                String[] data = lines.get(index++).trim().split(":");
                if (data.length != c) throw new IOException("Incorrect column count at row " + row);
                for (int col=0; col<c; col++) {
                    int value = Integer.parseInt(data[col].trim());
                    if (value != 0 && value != -1)
                        throw new IOException("Invalid occupancy value at " + row + "," + col);
                    loaded[row][col] = value;
                }
            }
            ArrayList<Point> starts = new ArrayList<>();
            if (index < lines.size() && lines.get(index).startsWith("DRONES")) index++;
            if (index < lines.size() && lines.get(index).startsWith("NUM DRONES")) {
                int n = readHeaderInteger(lines, index++, "NUM DRONES");
                if (n < 0 || n > r*c) throw new IOException("Invalid drone count");
                java.util.HashSet<Integer> unique = new java.util.HashSet<>();
                for (int i=0; i<n; i++) {
                    if (index >= lines.size()) throw new IOException("Missing drone entry " + i);
                    String[] pos = lines.get(index++).trim().split(":");
                    if (pos.length < 2) throw new IOException("Invalid drone coordinate");
                    int rr=Integer.parseInt(pos[0].trim()), cc=Integer.parseInt(pos[1].trim());
                    if (rr < 0 || rr >= r || cc < 0 || cc >= c || loaded[rr][cc] == -1)
                        throw new IOException("Drone cannot start at (" + rr + "," + cc + ")");
                    if (!unique.add(rr*c+cc)) throw new IOException("Duplicate starting position");
                    starts.add(new Point(rr,cc));
                }
            }
            // Commit only after every line has been validated.
            clearReplay();
            environment = loaded;
            numRows=r; numCols=c;
            translateX=0; translateY=0; scale=1;
            setWidth(numCols*cellSize); setHeight(numRows*cellSize);
            drones = new ArrayList<>();
            for (Point pos : starts)
                drones.add(new Drone(environment,pos.getRow(),pos.getCol(),sensorFoV,cameraFoV,cellSize,scale));
            checkWallType();
            setFreeTransform();
            render();
        } catch (Exception ex) {
            throw new IllegalArgumentException("Failed to open environment: " + ex.getMessage(), ex);
        }
    }

    private static int readHeaderInteger(java.util.List<String> lines, int index, String key) throws IOException {
        if (index >= lines.size()) throw new IOException("Missing " + key);
        String[] tokens = lines.get(index).trim().split(":");
        if (tokens.length < 2 || !tokens[0].trim().equals(key))
            throw new IOException("Expected " + key + " at line " + (index+1));
        try { return Integer.parseInt(tokens[1].trim()); }
        catch (NumberFormatException e) { throw new IOException("Invalid " + key, e); }
    }

    // =========================================================
    // WALL TYPE
    // =========================================================

    private String checkWallType(int i, int j) {
        String type = null;
        if (environment != null && i >= 0 && i < environment.length && j >= 0 && j < environment[0].length && environment[i][j] == -1) {
            type = "";


            // =================================================
            // EAST
            // =================================================

            String value = "0";
            if (j == environment[i].length - 1 || (j < environment[i].length - 1 && environment[i][j + 1] == 0))
                value = "1";
            type += value;


            // =================================================
            // SOUTH
            // =================================================

            value = "0";
            if (i == environment.length - 1 || (i < environment.length - 1 && environment[i + 1][j] == 0)) value = "1";


            type += value;


            // =================================================
            // WEST
            // =================================================

            value = "0";
            if (j == 0 || (j > 0 && environment[i][j - 1] == 0)) value = "1";
            type += value;


            // =================================================
            // NORTH
            // =================================================

            value = "0";
            if (i == 0 || (i > 0 && environment[i - 1][j] == 0)) value = "1";
            type += value;
        }


        return type;
    }


    private void checkWallType() {
        if (environment != null) {
            wallType = new String[environment.length][];
            for (int i = 0; i < environment.length; i++) {
                wallType[i] = new String[environment[i].length];
                for (int j = 0; j < environment[i].length; j++) wallType[i][j] = checkWallType(i, j);
            }
        }
    }


    // =========================================================
    // STATE
    // =========================================================

    protected State getState() {
        return state;
    }

    protected void setState(State state) {
        this.state = state;
    }

    protected void setFreeTransform() {
        state = State.DEFAULT;
        moveToI = -1;
        moveToJ = -1;
        lineToI = -1;
        lineToJ = -1;
        positionI = -1;
        positionJ = -1;
        draggingDrone = -1;
    }


    // =========================================================
    // RESET ENVIRONMENT
    // =========================================================

    protected void resetEnvironment() {
        clearReplay();
        numRows = MAX_NUM_ROWS;
        numCols = MAX_NUM_COLS; /* Reset camera. */
        translateX = 0.0;
        translateY = 0.0;
        scale = 1.0;
        setWidth(numCols * cellSize);
        setHeight(numRows * cellSize);
        environment = new int[numRows][numCols];
        wallType = new String[numRows][numCols];
        drones = new ArrayList<>();
        state = State.DEFAULT;
        moveToI = -1;
        moveToJ = -1;
        lineToI = -1;
        lineToJ = -1;
        positionI = -1;
        positionJ = -1;
        draggingDrone = -1;
        render();
    }


    // =========================================================
    // POINTER GESTURE HELPERS
    // =========================================================

    /**
     * Menyiapkan status gesture baru saat tombol mouse ditekan.
     * Setiap press baru membatalkan sisa flag dari gesture sebelumnya.
     */
    private void beginPointerGesture(MouseEvent mouseEvent) {
        mousePressActive = true;
        pressedMouseButton = mouseEvent.getButton();

        mousePressX = mouseEvent.getX();
        mousePressY = mouseEvent.getY();

        mouseAnchorX = mousePressX;
        mouseAnchorY = mousePressY;

        dragGestureDetected = false;
        zoomGestureDetected = false;
        suppressNextClick = false;
        panningCamera = false;
    }


    /**
     * Menandai gesture sebagai drag ketika pointer sudah bergerak
     * melewati ambang DRAG_THRESHOLD_PX.
     */
    private void updateDragGesture(double mouseX, double mouseY) {
        if (!mousePressActive || dragGestureDetected) return;

        double deltaX = mouseX - mousePressX;
        double deltaY = mouseY - mousePressY;

        if (Math.hypot(deltaX, deltaY) >= DRAG_THRESHOLD_PX) {
            dragGestureDetected = true;
            suppressNextClick = true;
        }
    }


    /**
     * Menghapus flag gesture setelah event MOUSE_CLICKED selesai diproses.
     */
    private void clearCompletedClickGesture() {
        dragGestureDetected = false;
        zoomGestureDetected = false;
        suppressNextClick = false;
    }


    // =========================================================
    // MOUSE CLICKED
    // =========================================================

    protected void handleMouseClicked(MouseEvent mouseEvent) {

        /*
         * ADD_DRONE, REMOVE_DRONE, dan OBSTACLE hanya boleh dijalankan
         * menggunakan klik kiri tunggal.
         *
         * Klik tengah tidak pernah menambah/menghapus drone karena tombol
         * tengah disediakan khusus untuk pan camera.
         */
        if (mouseEvent.getButton() != MouseButton.PRIMARY || mouseEvent.getClickCount() != 1) {
            mouseEvent.consume();
            return;
        }

        /*
         * MOUSE_CLICKED dapat tetap dikirim oleh JavaFX setelah rangkaian
         * press-drag-release tertentu. Karena itu, klik dibatalkan apabila
         * pada gesture yang sama pernah terjadi drag atau zoom.
         */
        if (suppressNextClick || dragGestureDetected || zoomGestureDetected) {
            clearCompletedClickGesture();
            mouseEvent.consume();
            return;
        }

        double mouseX = mouseEvent.getX();
        double mouseY = mouseEvent.getY();

        if (!isMouseInsideEnvironment(mouseX, mouseY)) {
            clearCompletedClickGesture();
            mouseEvent.consume();
            return;
        }

        int i = getRowFromMouse(mouseY);
        int j = getColumnFromMouse(mouseX);

        if (!isValidGridPosition(i, j)) {
            clearCompletedClickGesture();
            mouseEvent.consume();
            return;
        }


        // =====================================================
        // ADD / REMOVE OBSTACLE
        // =====================================================

        if (state.equals(State.OBSTACLE)) {
            clearReplay();
            if (environment[i][j] == 0) environment[i][j] = -1;
            else environment[i][j] = 0;
        }


        // =====================================================
        // ADD DRONE
        // =====================================================

        else if (state.equals(State.ADD_DRONE)) {
            clearReplay();
            Drone drone = new Drone(environment, i, j, sensorFoV, cameraFoV, cellSize, scale);
            drones.add(drone);
        }


        // =====================================================
        // REMOVE DRONE
        // =====================================================

        else if (state.equals(State.REMOVE_DRONE) && !drones.isEmpty()) {
            clearReplay();
            for (int k = drones.size() - 1; k >= 0; k--) {
                Drone drone = drones.get(k);
                if (drone.initialRowPosition == i && drone.initialColPosition == j) {
                    drones.remove(k);
                    break;
                }
            }
        }

        checkWallType();
        render();
        clearCompletedClickGesture();
        mouseEvent.consume();
    }


    // =========================================================
    // MOUSE SCROLL / ZOOM
    // =========================================================

    protected void handleMouseScroll(ScrollEvent scrollEvent) {

        /*
         * Apabila scroll terjadi ketika sebuah tombol mouse masih ditekan,
         * rangkaian gesture tersebut tidak boleh diinterpretasikan sebagai
         * klik ADD_DRONE atau REMOVE_DRONE ketika tombol dilepas.
         */
        if (mousePressActive) {
            zoomGestureDetected = true;
            suppressNextClick = true;
        }

        /*
         * =====================================================
         * 1. AMBIL POSISI POINTER MOUSE
         * =====================================================
         */
        final double mouseX = scrollEvent.getX();
        final double mouseY = scrollEvent.getY();

        /*
         * =====================================================
         * 2. SIMPAN SCALE LAMA
         * =====================================================
         */
        final double oldScale = scale;

        /*
         * =====================================================
         * 3. HITUNG SCALE BARU
         * =====================================================
         */
        double newScale;
        if (scrollEvent.getDeltaY() > 0) {
            newScale = oldScale * ZOOM_FACTOR;
        } else if (scrollEvent.getDeltaY() < 0) {
            newScale = oldScale / ZOOM_FACTOR;
        } else {
            return;
        }

        /*
         * =====================================================
         * 4. BATASI SCALE
         * =====================================================
         */
        newScale = Math.max(MIN_SCALE, Math.min(newScale, MAX_SCALE));

        if (Math.abs(newScale - oldScale) < 0.0000001) {
            scrollEvent.consume();
            return;
        }

        /*
         * =====================================================
         * 5. CARI WORLD POINT DI BAWAH POINTER
         * =====================================================
         */
        final double worldX = screenToWorldX(mouseX);
        final double worldY = screenToWorldY(mouseY);

        /*
         * =====================================================
         * 6. UPDATE SCALE
         * =====================================================
         */
        scale = newScale;

        /*
         * =====================================================
         * 7. HITUNG TRANSLATION BARU
         * =====================================================
         */
        translateX = mouseX - worldX * scale;
        translateY = mouseY - worldY * scale;

        render();
        scrollEvent.consume();
    }


    // =========================================================
    // MOUSE PRESSED
    // =========================================================

    protected void handleMousePressed(MouseEvent mouseEvent) {
        beginPointerGesture(mouseEvent);

        double mouseX = mouseEvent.getX();
        double mouseY = mouseEvent.getY();
        int i = getRowFromMouse(mouseY);
        int j = getColumnFromMouse(mouseX);


        // =====================================================
        // MIDDLE BUTTON: PAN CAMERA PADA SEMUA STATE
        // =====================================================

        if (pressedMouseButton == MouseButton.MIDDLE) {
            panningCamera = true;
            draggingDrone = -1;
            mouseEvent.consume();
            return;
        }


        // =====================================================
        // LINE MODE: HANYA KLIK KIRI
        // =====================================================

        if (pressedMouseButton == MouseButton.PRIMARY
                && (state.equals(State.LINE) || state.equals(State.REMOVE_LINE))) {

            if (moveToI == -1 && moveToJ == -1 && isMouseInsideEnvironment(mouseX, mouseY)) {
                moveToI = i;
                moveToJ = j;
                lineToI = -1;
                lineToJ = -1;
            }

            mouseEvent.consume();
            return;
        }


        // =====================================================
        // RESET DRAGGING DRONE
        // =====================================================

        draggingDrone = -1;


        // =====================================================
        // DRAG DRONE HANYA PADA STATE.DEFAULT + KLIK KIRI
        // =====================================================

        if (pressedMouseButton == MouseButton.PRIMARY
                && state.equals(State.DEFAULT)
                && isValidGridPosition(i, j)) {

            for (int k = 0; k < drones.size(); k++) {
                Drone drone = drones.get(k);
                if (i == drone.initialRowPosition && j == drone.initialColPosition) {
                    draggingDrone = k;
                    System.out.println("DRAG DRONE: " + k);
                    break;
                }
            }
        }

        /*
         * Pada ADD_DRONE dan REMOVE_DRONE, klik kiri tidak memulai pan
         * ataupun drag drone. Aksinya baru diputuskan pada MOUSE_CLICKED
         * setelah dipastikan tidak ada drag atau zoom.
         */
        mouseEvent.consume();
    }


    // =========================================================
    // MOUSE MOVED
    // =========================================================

    protected void handleMouseMoved(MouseEvent mouseEvent) {
        if (state.equals(State.OBSTACLE) || state.equals(State.LINE) || state.equals(State.REMOVE_LINE)) {
            double mouseX = mouseEvent.getX();
            double mouseY = mouseEvent.getY();

            if (isMouseInsideEnvironment(mouseX, mouseY)) {
                int i = getRowFromMouse(mouseY);
                int j = getColumnFromMouse(mouseX);

                if (isValidGridPosition(i, j)) {
                    positionI = i;
                    positionJ = j;
                    render();
                }
            }
        }
    }


    // =========================================================
    // MOUSE DRAGGED
    // =========================================================

    protected void handleMouseDragged(MouseEvent mouseEvent) {
        double mouseX = mouseEvent.getX();
        double mouseY = mouseEvent.getY();

        updateDragGesture(mouseX, mouseY);


        // =====================================================
        // PAN CAMERA DENGAN TOMBOL TENGAH PADA SEMUA STATE
        // =====================================================

        if (panningCamera && pressedMouseButton == MouseButton.MIDDLE) {
            double deltaX = mouseX - mouseAnchorX;
            double deltaY = mouseY - mouseAnchorY;

            translateX += deltaX;
            translateY += deltaY;

            mouseAnchorX = mouseX;
            mouseAnchorY = mouseY;

            render();
            mouseEvent.consume();
            return;
        }

        int i = getRowFromMouse(mouseY);
        int j = getColumnFromMouse(mouseX);


        // =====================================================
        // OBSTACLE / LINE: HANYA KLIK KIRI
        // =====================================================

        if (pressedMouseButton == MouseButton.PRIMARY
                && (state.equals(State.OBSTACLE)
                || state.equals(State.LINE)
                || state.equals(State.REMOVE_LINE))) {

            if (moveToI != -1 && moveToJ != -1 && isMouseInsideEnvironment(mouseX, mouseY)) {
                positionI = i;
                positionJ = j;

                if (state.equals(State.LINE) || state.equals(State.REMOVE_LINE)) {
                    lineToI = i;
                    lineToJ = j;
                }
            }

            render();
            mouseEvent.consume();
            return;
        }


        // =====================================================
        // DRAG DRONE HANYA PADA STATE.DEFAULT
        // =====================================================

        if (pressedMouseButton == MouseButton.PRIMARY
                && state.equals(State.DEFAULT)
                && dragGestureDetected
                && draggingDrone >= 0
                && draggingDrone < drones.size()
                && isValidGridPosition(i, j)) {

            clearReplay();
            drones.get(draggingDrone).initialRowPosition = i;
            drones.get(draggingDrone).initialColPosition = j;
            render();
        }

        /*
         * Pada ADD_DRONE / REMOVE_DRONE, primary drag sengaja tidak melakukan
         * perubahan apa pun. Karena dragGestureDetected sudah true, event
         * MOUSE_CLICKED setelah release akan dibatalkan.
         */
        mouseEvent.consume();
    }


    // =========================================================
    // MOUSE RELEASED
    // =========================================================

    protected void handleMouseReleased(MouseEvent mouseEvent) {

        if (pressedMouseButton == MouseButton.PRIMARY
                && moveToI != -1
                && moveToJ != -1
                && lineToI != -1
                && lineToJ != -1) {

            clearReplay();
            drawLineOnGrid(moveToI, moveToJ, lineToI, lineToJ, state.equals(State.LINE));
            checkWallType();
        }


        // =====================================================
        // RESET LINE STATE
        // =====================================================

        moveToI = -1;
        moveToJ = -1;
        lineToI = -1;
        lineToJ = -1;
        draggingDrone = -1;

        /*
         * Simpan informasi bahwa gesture bukan klik murni.
         * Flag ini akan diperiksa oleh handleMouseClicked().
         */
        if (dragGestureDetected || zoomGestureDetected || panningCamera) {
            suppressNextClick = true;
        }

        mousePressActive = false;
        panningCamera = false;
        pressedMouseButton = MouseButton.NONE;

        render();
        mouseEvent.consume();
    }


    // =========================================================
    // DRAW LINE ON GRID
    // =========================================================

    private void drawLineOnGrid(int row0, int col0, int row1, int col1, boolean addObstacle) {
        int x0 = col0, y0 = row0, x1 = col1, y1 = row1, dx = Math.abs(x1 - x0), dy = Math.abs(y1 - y0), sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1, err = dx - dy;
        while (true) {
            if (isValidGridPosition(y0, x0)) if (addObstacle) environment[y0][x0] = -1;
            else environment[y0][x0] = 0;
            if (x0 == x1 && y0 == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y0 += sy;
            }
        }
    }

    // =========================================================
    // UPDATE
    // =========================================================
    protected void update() {
        if (!playbackPlaying || lastExploration == null) return;
        long now = System.nanoTime();
        if (now - lastPlaybackTick < PLAYBACK_INTERVAL_NS) return;
        lastPlaybackTick = now;
        if (playbackFrame < totalPlaybackFrames()) setPlaybackFrame(playbackFrame + 1);
        if (playbackFrame >= totalPlaybackFrames()) playbackPlaying = false;
    }

    // =========================================================
    // RENDER
    // =========================================================
    protected void render() {
        if (environment == null) return;
        halfCellSize = 0.5 * cellSize;
        GraphicsContext gc = this.getGraphicsContext2D();
        gc.setImageSmoothing(true);
        gc.setFontSmoothingType(FontSmoothingType.LCD); /* Clear seluruh Canvas dalam SCREEN SPACE. clearRect dilakukan sebelum transformasi kamera. */
        gc.clearRect(0, 0, getWidth(), getHeight());
        gc.save();
        // =====================================================
        // WORLD TRANSFORMATION
        //
        // Screen = World * scale + translate
        // =====================================================

        gc.translate(translateX, translateY);
        gc.scale(scale, scale);


        // =====================================================
        // BEGIN WORLD VISUALIZATION
        // =====================================================


        // =====================================================
        // BACKGROUND
        // =====================================================

        gc.setFill(Color.valueOf("#F7F7F7"));


        /*
         * Gunakan ukuran WORLD.
         *
         * Jangan menggunakan getWidth()/getHeight()
         * setelah transformasi scale.
         */

        gc.fillRect(0, 0, numCols * cellSize, numRows * cellSize);


        // =====================================================
        // HORIZONTAL GRID
        // =====================================================

        for (int i = 0; i <= numRows; i++) {
            gc.setLineWidth(0.2);
            gc.setStroke(Color.valueOf("#929AAB"));
            if (i % 10 == 0) {
                gc.setLineWidth(0.6);
                gc.setStroke(Color.valueOf("#929AAB"));
            }
            gc.strokeLine(0, i * cellSize, numCols * cellSize, i * cellSize);
        }


        // =====================================================
        // VERTICAL GRID
        // =====================================================

        for (int i = 0; i <= numCols; i++) {
            gc.setLineWidth(0.2);
            gc.setStroke(Color.valueOf("#929AAB"));
            if (i % 10 == 0) {
                gc.setLineWidth(0.6);
                gc.setStroke(Color.valueOf("#929AAB"));
            }
            gc.strokeLine(i * cellSize, 0, i * cellSize, numRows * cellSize);
        }


        // =====================================================
        // DRAW OBSTACLE SHADOW
        // =====================================================

        gc.setGlobalAlpha(0.15);
        for (int i = 0; i < environment.length; i++) {
            for (int j = 0; j < environment[i].length; j++) {
                if (environment[i][j] == -1) {
                    double xo = j * cellSize, yo = i * cellSize;
                    gc.setFill(Color.valueOf("#505050"));
                    gc.fillRect(xo + 3, yo + 3, cellSize, cellSize);
                }
            }
        }
        gc.setGlobalAlpha(1.0);


        // =====================================================
        // DRAW WALL
        // =====================================================

        for (int i = 0; i < environment.length; i++) {
            for (int j = 0; j < environment[i].length; j++) {
                if (environment[i][j] == -1) {
                    double xo = j * cellSize, yo = i * cellSize;
                    gc.setFill(Color.valueOf("#03A6A1"));
                    gc.fillRect(xo, yo, cellSize, cellSize);
                }
            }
        }


        // =====================================================
        // HATCHING LINES
        // =====================================================

        for (int i = 0; i < environment.length; i++) {
            for (int j = 0; j < environment[i].length; j++) {
                if (environment[i][j] == -1) {
                    double xo = j * cellSize, yo = i * cellSize;
                    gc.setStroke(Color.WHITE);
                    gc.setLineWidth(0.3);
                    gc.strokeLine(xo, yo, xo + cellSize, yo + cellSize);
                    gc.strokeLine(xo + halfCellSize, yo, xo + cellSize, yo + cellSize - halfCellSize);
                    gc.strokeLine(xo, yo + halfCellSize, xo + cellSize - halfCellSize, yo + cellSize);


                    // =================================================
                    // WALL TYPE
                    // =================================================

                    if (wallType != null && i < wallType.length && wallType[i] != null && j < wallType[i].length && wallType[i][j] != null && wallType[i][j].length() >= 4) {
                        gc.setLineWidth(0.6);
                        String type = wallType[i][j];


                        // EAST
                        char value = type.charAt(0);
                        if (value == '1') gc.strokeLine(xo + cellSize, yo, xo + cellSize, yo + cellSize);


                        // SOUTH
                        value = type.charAt(1);
                        if (value == '1') gc.strokeLine(xo, yo + cellSize, xo + cellSize, yo + cellSize);


                        // WEST
                        value = type.charAt(2);
                        if (value == '1') gc.strokeLine(xo, yo, xo, yo + cellSize);


                        // NORTH
                        value = type.charAt(3);
                        if (value == '1') gc.strokeLine(xo, yo, xo + cellSize, yo);
                    }
                }
            }
        }


        // =====================================================
        // CELL CORNERS
        // =====================================================
        gc.setGlobalAlpha(1.0);
        for (int i = 0; i <= numRows; i++) {
            for (int j = 0; j <= numCols; j++) {
                double xo = j * cellSize, yo = i * cellSize;
                gc.setFill(Color.valueOf("#929AAB"));
                gc.fillOval(xo - 0.4, yo - 0.4, 0.8, 0.8);
            }
        }
        gc.setGlobalAlpha(1.0);


        // =====================================================
        // DRAW PREVIEW LINE
        // =====================================================

        if (moveToI != -1 && moveToJ != -1 && lineToI != -1 && lineToJ != -1) {
            double x0 = moveToJ * cellSize + halfCellSize, y0 = moveToI * cellSize + halfCellSize, x1 = lineToJ * cellSize + halfCellSize, y1 = lineToI * cellSize + halfCellSize;
            gc.setStroke(Color.valueOf("#03A6A1"));
            if (state.equals(State.REMOVE_LINE)) gc.setStroke(Color.RED);
            gc.setLineWidth(0.8);
            gc.strokeLine(x0, y0, x1, y1);
        }


        // Unknown cells are DARK; locally sensed cells are LIGHT.
        // The visual union never enters the algorithm or agent local maps.
        if (lastExploration != null) {
            int[][] knownAt = lastExploration.getFirstObservedAt();
            boolean[][] exploreArea = lastExploration.getMustExploreMask();
            gc.setFill(Color.valueOf("#263241"));
            gc.setGlobalAlpha(0.76);
            for (int r=0; r<numRows; r++) for (int c=0; c<numCols; c++) {
                if (exploreArea[r][c] && environment[r][c] != -1 &&
                    (knownAt[r][c] == -1 || knownAt[r][c] > playbackFrame))
                    gc.fillRect(c*cellSize,r*cellSize,cellSize,cellSize);
            }
            gc.setGlobalAlpha(1.0);
        }

        // EXPLORED CELLS: faint historical visitation overlay, revealed during replay.
        // This is visualization only; no occupancy or algorithm state is modified.
        gc.setFill(Color.valueOf("#79CDB2"));
        gc.setGlobalAlpha(0.20);
        for (Drone drone : drones) {
            if (drone.path == null) continue;
            int last = Math.min(Math.max(drone.frameIndex, 0), drone.path.size() - 1);
            for (int k = 0; k <= last; k++) {
                Point visited = drone.path.get(k);
                gc.fillRect(visited.getCol() * cellSize, visited.getRow() * cellSize, cellSize, cellSize);
            }
        }
        gc.setGlobalAlpha(1.0);

        // =====================================================
        // DRAW PATH OF DRONES
        // =====================================================

        if (drones != null && !drones.isEmpty()) {
            for (int i = 0; i < drones.size(); i++) {
                ArrayList<Point> path = drones.get(i).path;
                if (path != null && !path.isEmpty() && path.size() > 1) {
                    for (int k = 1; k <= Math.min(drones.get(i).frameIndex, path.size() - 1); k++) {
                        Point point0 = path.get(k - 1), point1 = path.get(k);
                        double x0 = (double) point0.getCol() * cellSize + halfCellSize, y0 = (double) point0.getRow() * cellSize + halfCellSize, x1 = (double) point1.getCol() * cellSize + halfCellSize, y1 = (double) point1.getRow() * cellSize + halfCellSize;
                        gc.setStroke(Color.valueOf("#D62828"));
                        gc.setLineWidth(0.8);
                        gc.strokeLine(x0, y0, x1, y1);
                    }
                }
            }
        }


        // =====================================================
        // DRAW DRONES
        // =====================================================
        if (drones != null && !drones.isEmpty()) for (int i = 0; i < drones.size(); i++) drones.get(i).draw(gc);


        // =====================================================
        // DRAW MOUSE POSITION
        // =====================================================
        if ((state.equals(State.OBSTACLE) || state.equals(State.LINE) || state.equals(State.REMOVE_LINE)) && isValidGridPosition(positionI, positionJ)) {
            double px = positionJ * cellSize + halfCellSize, py = positionI * cellSize + halfCellSize;
            String textPosition = "(" + (positionI + 1) + "," + (positionJ + 1) + ")";
            gc.setFont(Font.font("Arial", 8));
            gc.setFill(Color.BLACK);
            gc.fillText(textPosition, px, py + 2 + 3 * cellSize);
        }


        // =====================================================
        // END WORLD VISUALIZATION
        // =====================================================

        gc.restore();
    }


    // =========================================================
    // EXPLORATION
    // =========================================================
    private void getBorders() {
        MIN_ROW = numRows - 1;
        MIN_COL = numCols - 1;
        MAX_ROW = 0;
        MAX_COL = 0;
        for (int i = 0; i < environment.length; i++)
            for (int j = 0; j < environment[i].length; j++)
                if (environment[i][j] == -1) {
                    if (i < MIN_ROW) MIN_ROW = i;
                    if (j < MIN_COL) MIN_COL = j;
                    if (i > MAX_ROW) MAX_ROW = i;
                    if (j > MAX_COL) MAX_COL = j;
                }
    }

    private boolean isDroneValid(int initialRowPosition, int initialColPosition) {
        return initialRowPosition >= MIN_ROW && initialRowPosition <= MAX_ROW && initialColPosition >= MIN_COL && initialColPosition <= MAX_COL;
    }

    private ArrayList<Drone> getActiveDrones() {
        int activeDrone = 0;
        for (int i = drones.size() - 1; i >= 0; i--) {
            Drone drone = drones.get(i);
            if (isDroneValid(drone.initialRowPosition, drone.initialColPosition)) activeDrone++;
            else drones.remove(i);
        }

        // =====================================================
        // SET DRONE NAME
        // =====================================================
        if (drones != null && !drones.isEmpty())
            for (int i = 0; i < drones.size(); i++) drones.get(i).setName("D" + (i + 1));
        return drones;
    }


    // =========================================================
    // RUN
    // =========================================================
    protected void run() {
        // Simulate the full configured environment without silently removing valid agents
        // merely because they fall outside the bounding rectangle of drawn walls.
        MIN_ROW = 0;
        MIN_COL = 0;
        MAX_ROW = numRows - 1;
        MAX_COL = numCols - 1;
        for (int i = 0; i < drones.size(); i++) drones.get(i).setName("D" + (i + 1));
        clearReplay();
        if (!drones.isEmpty()) {
            lastExploration = new DecentralizedExploration(environment, drones, MIN_ROW, MIN_COL, MAX_ROW, MAX_COL);
            setPlaybackFrame(0);
            playbackPlaying = true;
            lastPlaybackTick = System.nanoTime();
        }
        render();
        System.out.println("TRACE RUN");
    }
}