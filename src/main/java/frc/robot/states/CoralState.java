package frc.robot.states;

import frc.robot.constants.CoralConstants;

public enum CoralState {
    NONE(0, 0, false),
    INTAKE(CoralConstants.INCH_INTAKE_SPEED, 0, false),
    REVERSE(CoralConstants.REVERSE_SPEED, 0, false),
    INDEX(CoralConstants.INDEX_SPEED, 0, false),
    READY(0, 0, false),
    LASER_EXIT_ERROR(-1, 0, true),
    LASER_ENTRY_ERROR(-1, 0, true),
    L1(CoralConstants.L1_SPEED, CoralConstants.SPEED_DIFFERENCE, false),
    L23(CoralConstants.L23_SPEED, 0, false),
    L4(CoralConstants.L4_SPEED, 0, false),
    INCH_INTAKE(CoralConstants.INCH_INTAKE_SPEED, 0, false),
    INCH_REVERSE(-CoralConstants.INCH_INTAKE_SPEED, 0, false);

    private double speed;
    private double speedDiff;
    private boolean isError;

    CoralState(double speed, double speedDiff, boolean isError) {
        this.speed = speed;
        this.speedDiff = speedDiff;
        this.isError = isError;
    }

    public double getSpeed() {
        return speed;
    }

    public double getSpeedDiff() {
        return speedDiff;
    }

    public boolean isError() {
        return isError;
    }
}
