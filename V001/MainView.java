package swarmdrone.decentralizedswarmcoverage;

import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.geometry.Insets;
import javafx.geometry.Orientation;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.*;
import javafx.scene.image.Image;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.ScrollEvent;
import javafx.scene.layout.*;
import javafx.scene.paint.Color;
import javafx.scene.shape.SVGPath;
import javafx.stage.FileChooser;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;

import java.io.File;

public class MainView extends Application {

    // BUTTON
    private final ToggleGroup toolGroup = new ToggleGroup();
    private final Button btnNew = initializeButton(SVGIcon.newEnv, 1.0, 1.0, "New Environment");// VISUALIZATION
    private final Button btnOpen = initializeButton(SVGIcon.openEnv, 1.0, 1.0, "Open Environment");// VISUALIZATION
    private final Button btnSave = initializeButton(SVGIcon.saveEnv, 1.0, 1.0, "Save Environment");// VISUALIZATION
    private final ToggleButton btnTransform = initializeToggleButton(SVGIcon.transform, 1.0, 1.0, "Transform");// VISUALIZATION
    private final ToggleButton btnAddDrone = initializeToggleButton(SVGIcon.addDrone, 1.2, 1.2, "Add Drone");// VISUALIZATION
    private final ToggleButton btnRemoveDrone = initializeToggleButton(SVGIcon.removeDrone, 1.2, 1.2, "Remove Drone");// VISUALIZATION
    private final ToggleButton btnWall = initializeToggleButton(SVGIcon.wall, 1.0, 1.0, "Wall");// VISUALIZATION
    private final ToggleButton btnDrawLine = initializeToggleButton(SVGIcon.drawLine, 1.0, 1.0, "Draw Line");// VISUALIZATION
    private final ToggleButton btnRemoveLine = initializeToggleButton(SVGIcon.removeLine, 1.0, 1.0, "Remove Line");// VISUALIZATION
    private final Button btnRun = initializeButton(SVGIcon.run, 1.0, 1.0, "Run");// VISUALIZATION
    private final Button btnPlayPause = initializeButton(SVGIcon.playPause, 1.0, 1.0, "Play - Pause");// VISUALIZATION
    private final Button btnReplay = initializeButton(SVGIcon.replay, 1.0, 1.0, "Replay");// VISUALIZATION
    private final int MAX_NUM_ROWS = 200;// GRID
    private final int MAX_NUM_COLS = 400;// GRID
    private final int cellSize = 10;// GRID
    private final int numRows = MAX_NUM_ROWS;// GRID
    private final int numCols = MAX_NUM_COLS;// GRID
    private final int[][] environment = new int[numRows][numCols];// GRID
    private final String help = "DRONE SIMULATOR";
    private Label statusLabel = new Label("");// VISUALIZATION
    private Label statelabel = new Label("");// VISUALIZATION
    private FileChooser fileChooser = new FileChooser();// VISUALIZATION
    private Stage primaryStage = null;
    // VISUALIZATION
    private ProgressIndicator progressIndicator;// VISUALIZATION
    private CanvasView canvas = null;
    private StringBuffer logInfo = new StringBuffer();//LOG

    // Create button
    private Button initializeButton(String svgPathData, double scaleX, double scaleY, String tooltip) {
        SVGPath path = new SVGPath();
        path.setContent(svgPathData);
        path.setFill(Color.valueOf("#393E46"));
        path.setScaleX(scaleX);
        path.setScaleY(scaleY);
        Button btn = new Button();
        btn.setGraphic(path);
        btn.setTooltip(new Tooltip(tooltip));
        btn.setStyle("-fx-background-color: transparent;");
        btn.setOnMouseEntered(e -> path.setFill(Color.valueOf("#0096FF")));
        btn.setOnMouseExited(e -> path.setFill(Color.valueOf("#393E46")));
        return btn;
    }

    // Create ToggleButton
    private ToggleButton initializeToggleButton(String svgPathData, double scaleX, double scaleY, String tooltip) {
        SVGPath path = new SVGPath();
        path.setContent(svgPathData);
        path.setFill(Color.valueOf("#393E46"));
        path.setScaleX(scaleX);
        path.setScaleY(scaleY);

        ToggleButton btn = new ToggleButton();
        btn.setGraphic(path);
        btn.setTooltip(new Tooltip(tooltip));
        btn.setStyle("-fx-background-color: transparent;");

        // Hover
        btn.setOnMouseEntered(e -> {
            if (!btn.isSelected()) {
                path.setFill(Color.valueOf("#0096FF"));
            }
        });

        btn.setOnMouseExited(e -> {
            if (!btn.isSelected()) {
                path.setFill(Color.valueOf("#393E46"));
            }
        });

        // Selected state
        btn.selectedProperty().addListener((obs, oldVal, isSelected) -> {
            if (isSelected) {
                path.setFill(Color.valueOf("#0096FF")); // warna aktif
            } else {
                path.setFill(Color.valueOf("#393E46")); // kembali normal
            }
        });

        return btn;
    }

    private void initializeToggleGroup() {
        btnTransform.setToggleGroup(toolGroup);
        btnAddDrone.setToggleGroup(toolGroup);
        btnRemoveDrone.setToggleGroup(toolGroup);
        btnWall.setToggleGroup(toolGroup);
        btnDrawLine.setToggleGroup(toolGroup);
        btnRemoveLine.setToggleGroup(toolGroup);

        // default tool
        btnTransform.setSelected(true);

        // aksi untuk tombol di toggle group
        toolGroup.selectedToggleProperty().addListener((obs, oldToggle, newToggle) -> {
            if (newToggle == null && oldToggle != null) {
                oldToggle.setSelected(true);
                return;
            }

            if (newToggle == btnTransform) {
                canvas.setFreeTransform();
            } else if (newToggle == btnAddDrone) {
                canvas.setState(State.ADD_DRONE);
            } else if (newToggle == btnRemoveDrone) {
                canvas.setState(State.REMOVE_DRONE);
            } else if (newToggle == btnWall) {
                canvas.setState(State.OBSTACLE);
            } else if (newToggle == btnDrawLine) {
                canvas.setState(State.LINE);
            } else if (newToggle == btnRemoveLine) {
                canvas.setState(State.REMOVE_LINE);
            }

            checkState();
        });
    }

    private CanvasView getCanvas() {
        if (canvas == null) {
            canvas = new CanvasView(environment, cellSize);
            canvas.setOnMousePressed(this::handleMousePressed);
            canvas.setOnMouseDragged(this::handleMouseDragged);
            canvas.setOnScroll(this::handleMouseScroll);
            canvas.setOnMouseClicked(this::handleMouseClicked);
            canvas.setOnMouseReleased(this::handleMouseReleased);
            canvas.setOnMouseMoved(this::handleMouseMoved);
        }
        return canvas;
    }

    private void handleMouseMoved(MouseEvent mouseEvent) {
        canvas.handleMouseMoved(mouseEvent);
    }

    private void handleMouseReleased(MouseEvent mouseEvent) {
        canvas.handleMouseReleased(mouseEvent);
    }

    private void handleMouseClicked(MouseEvent mouseEvent) {
        canvas.handleMouseClicked(mouseEvent);

    }

    private void handleMouseScroll(ScrollEvent scrollEvent) {
        canvas.handleMouseScroll(scrollEvent);
    }

    private void handleMouseDragged(MouseEvent mouseEvent) {
        canvas.handleMouseDragged(mouseEvent);
    }

    private void handleMousePressed(MouseEvent mouseEvent) {
        canvas.handleMousePressed(mouseEvent);
    }

    private void handleButtonOpen(ActionEvent actionEvent) {
        fileChooser = new FileChooser();
        fileChooser.setTitle("Open Environment");

        // Set initial directory "Documents"
        File documentsDir = new File(System.getProperty("user.home"), "Documents");
        if (documentsDir.exists()) {
            fileChooser.setInitialDirectory(documentsDir);
        }

        // Opsional: filter jenis file
        fileChooser.getExtensionFilters().addAll(
                new FileChooser.ExtensionFilter("Data", "*.sc"),
                new FileChooser.ExtensionFilter("All Files", "*.*")
        );

        File selectedFile = fileChooser.showOpenDialog(primaryStage);
        if (selectedFile != null) {
            // System.out.println("Selected File: " + selectedFile.getAbsolutePath());
            canvas.readFromFile(selectedFile);
            checkState();
        } else {
            // System.out.println("Tidak ada file dipilih.");
        }
    }


    private void handleButtonSave(ActionEvent actionEvent) {
        fileChooser = new FileChooser();
        fileChooser.setTitle("Save Environment");

        // Set initial directory "Documents"
        File documentsDir = new File(System.getProperty("user.home"), "Documents");
        if (documentsDir.exists()) {
            fileChooser.setInitialDirectory(documentsDir);
        }

        // Opsional: filter jenis file
        fileChooser.getExtensionFilters().addAll(
                new FileChooser.ExtensionFilter("Data", "*.sc"),
                new FileChooser.ExtensionFilter("All Files", "*.*")
        );

        File selectedFile = fileChooser.showSaveDialog(primaryStage);
        if (selectedFile != null) {
            // System.out.println("Selected File: " + selectedFile.getAbsolutePath());
            canvas.writeToFile(selectedFile);
            checkState();
        } else {
            // System.out.println("Tidak ada file dipilih.");
        }
    }

    private void handleButtonRun(ActionEvent actionEvent) {
        canvas.setState(State.DEFAULT);
        checkState();
        canvas.run();
    }

    private void handleButtonPlayPause(ActionEvent actionEvent) {
        //
        canvas.setState(State.DEFAULT);
        checkState();
    }

    private void handleButtonReplay(ActionEvent actionEvent) {
        //
        canvas.setState(State.DEFAULT);
        checkState();
    }

    private void handleButtonResetEnvironment(ActionEvent actionEvent) {
        canvas.resetEnvironment();
        checkState();
    }

    private void checkState() {
        if (canvas.getState().equals(State.DEFAULT)) {
            statelabel.setText("STATE: " + "FREE TRANSFORM" + "\t-\t");
            btnTransform.setSelected(true);
        } else if (canvas.getState().equals(State.ADD_DRONE)) {
            statelabel.setText("STATE: " + "ADD DRONE" + "\t-\t");
            btnAddDrone.setSelected(true);
        } else if (canvas.getState().equals(State.REMOVE_DRONE)) {
            statelabel.setText("STATE: " + "REMOVE DRONE" + "\t-\t");
            btnRemoveDrone.setSelected(true);
        } else if (canvas.getState().equals(State.OBSTACLE)) {
            statelabel.setText("STATE: " + "DRAW OBSTACLE" + "\t-\t");
            btnWall.setSelected(true);
        } else if (canvas.getState().equals(State.LINE)) {
            statelabel.setText("STATE: " + "DRAW LINE" + "\t-\t");
            btnDrawLine.setSelected(true);
        } else if (canvas.getState().equals(State.REMOVE_LINE)) {
            statelabel.setText("STATE: " + "REMOVE LINE" + "\t-\t");
            btnRemoveLine.setSelected(true);
        }
    }

    @Override
    public void start(Stage stage) throws Exception {
        primaryStage = stage;

        // === Sidebar ===
        VBox sidebar = new VBox(2);
        sidebar.getStyleClass().add("sidebar");
        sidebar.setPadding(new Insets(4));

        // button
        initializeToggleGroup();
        // set on action for buttons
        btnOpen.setOnAction(this::handleButtonOpen);
        btnSave.setOnAction(this::handleButtonSave);
        btnRun.setOnAction(this::handleButtonRun);
        btnPlayPause.setOnAction(this::handleButtonPlayPause);
        btnReplay.setOnAction(this::handleButtonReplay);
        btnNew.setOnAction(this::handleButtonResetEnvironment);
        VBox topButtons = new VBox(2, btnNew, btnOpen, btnTransform, btnAddDrone, btnRemoveDrone, btnWall, btnDrawLine, btnRemoveLine, btnSave, btnRun, btnPlayPause, btnReplay);
        topButtons.setAlignment(Pos.TOP_CENTER);

        progressIndicator = new ProgressIndicator(0);
        progressIndicator.setVisible(false);
        statusLabel = new Label("");
        statelabel = new Label("");

        VBox bottomButtons = new VBox(2, progressIndicator);
        bottomButtons.setAlignment(Pos.BOTTOM_CENTER);
        VBox.setVgrow(topButtons, Priority.ALWAYS);
        sidebar.getChildren().addAll(topButtons, bottomButtons);

        // === Center Pane ===
        BorderPane centerPane = new BorderPane();
        centerPane.getStyleClass().add("center-pane");
        StackPane canvasPane = new StackPane();

        getCanvas();
        canvas.render();
        canvasPane.getChildren().add(canvas);

        // Scroll Pane
        ScrollPane scrollPane = new ScrollPane(canvasPane);
        scrollPane.setFitToWidth(true);
        scrollPane.setFitToHeight(true);
        scrollPane.setHbarPolicy(ScrollPane.ScrollBarPolicy.ALWAYS);
        scrollPane.setVbarPolicy(ScrollPane.ScrollBarPolicy.ALWAYS);
        scrollPane.setPrefHeight(900);

        // === Log Area ===
        logInfo = new StringBuffer();
        logInfo.append("Simulator start...\n" + help);

        // === Log Area ===
        TextArea logArea = new TextArea();
        logArea.setPrefRowCount(3);
        logArea.setWrapText(false);
        logArea.setEditable(true);
        logArea.setBorder(null);
        logArea.setText("Simulator start...\n" + help);

        // Bungkus dalam ScrollPane
        ScrollPane logScrollPane = new ScrollPane(logArea);
        logScrollPane.setFitToWidth(true);
        logScrollPane.setFitToHeight(true);
        logScrollPane.setHbarPolicy(ScrollPane.ScrollBarPolicy.ALWAYS);
        logScrollPane.setVbarPolicy(ScrollPane.ScrollBarPolicy.ALWAYS);


        // === Status Bar ===
        statusLabel = new Label("Ready.");
        statusLabel.getStyleClass().add("status-label");
        statelabel = new Label("STATE: " + canvas.getState().toString() + "\t-\t");
        statelabel.getStyleClass().add("status-label");
        HBox statusBar = new HBox(statelabel, statusLabel);
        statusBar.getStyleClass().add("status-bar");
        statusBar.setPadding(new Insets(5));

        // SplitPane vertikal
        SplitPane splitPane = new SplitPane(scrollPane, logArea);
        splitPane.setOrientation(Orientation.VERTICAL);
        splitPane.setDividerPositions(0.90);

        // === Bottom (log + status) ===
        VBox bottomBox = new VBox(statusBar);

        centerPane.setCenter(splitPane);
        centerPane.setBottom(bottomBox);

        // === Layout ===
        BorderPane root = new BorderPane();
        //root.setTop(topBar);
        root.setLeft(sidebar);
        root.setCenter(centerPane);
        Scene scene = new Scene(root, 1000, 600);
        scene.getStylesheets().add(CSS.style);

        /* STRART the ENGINE */
        AnimationTimer engine = new AnimationTimer() {
            @Override
            public void handle(long l) {
                // UPDATE
                canvas.update();

                // RENDER
                canvas.render();
            }
        };
        engine.start();

        // stage
        primaryStage.setTitle("Robotics & Mechatronics Laboratory - Mechanical Engineering - Hasanuddin University");
        primaryStage.setMaximized(true);
        primaryStage.setScene(scene);
        primaryStage.show();
        primaryStage.setOnCloseRequest(new EventHandler<WindowEvent>() {
            @Override
            public void handle(WindowEvent windowEvent) {
                Platform.exit();
                System.exit(0);
            }
        });
        primaryStage.getIcons().add(new Image(this.getClass().getResourceAsStream("icon.png")));
    }
}