package swarmdrone.decentralizedswarmcoverage;

public class CSS {
    public static final String style = "data:text/css," +
            /* ROOT */
            ".root {-fx-background-color: #EEEEEE;}" +

            /* Title Bar */
            ".title-bar { -fx-background-color: #27282b; }" +
            ".title-label { -fx-text-fill: #dfe1e5; -fx-font-size: 14; }" +

            /* Control Buttons */
            ".control-btn { -fx-background-color: transparent; -fx-text-fill: white; -fx-font-size: 12px; }" +
            ".control-btn:hover { -fx-background-color: #555; -fx-text-fill: white; }" +

            /* Info Bar */
            ".info-bar { -fx-background-color: #2b2d30; }" +
            ".info-label { -fx-text-fill: #8c8a8c; -fx-font-size: 10; }" +

            /* Sidebar */
            ".sidebar { -fx-background-color: #EEEEEE; -fx-pref-width: 10; }" +

            /* Canvas Pane */
            ".canvas-pane { -fx-background-color: #F7F7F7; }" +

            /* Center Pane */
            ".center-pane { " +
            "-fx-border-color: #929AAB;" +
            "-fx-border-width: 1;" +
            "-fx-border-radius: 2;" +
            "-fx-background-color: white;" +
            "-fx-border-insets: 0;" +
            "-fx-background-insets: 0;" +
            "}" +

            /* ScrollPane */
            ".scrollpane { -fx-background-color: #929AAB; }" + /*border center*/
            ".scrollpane .scroll-bar { -fx-background-color: #EEEEEE; }" +
            ".scrollpane .scroll-bar .thumb { -fx-background-color: #555555; -fx-background-radius: 4; }" +
            ".scrollpane .scroll-bar .thumb:hover { -fx-background-color: #777777; }" +
            ".scrollpane .increment-button, .scrollpane .decrement-button { -fx-background-color: #EEEEEE; }" +
            ".scrollpane .increment-arrow, .scrollpane .decrement-arrow { -fx-background-color: transparent; }" +
            ".scrollpane .corner { -fx-background-color: #929AAB; }" +

            /* SplitPane */
            ".split-pane { -fx-background-color: #EEEEEE; }" +
            ".split-pane-divider { -fx-background-color: #929AAB; }" +

            /* Status Bar */
            ".status-bar { -fx-background-color: #EEEEEE; }" +
            ".status-label { -fx-text-fill: #393E46; }";
}