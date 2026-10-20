package swarmdrone.decentralizedswarmcoverage_ijrcs;

import java.util.Objects;

/** Immutable integer grid coordinate (row, column). */
public final class Point {
    private final int row;
    private final int col;

    public Point(int row, int col) {
        this.row = row;
        this.col = col;
    }

    public int getRow() { return row; }
    public int getCol() { return col; }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (!(obj instanceof Point other)) return false;
        return row == other.row && col == other.col;
    }

    @Override
    public int hashCode() { return Objects.hash(row, col); }
}
