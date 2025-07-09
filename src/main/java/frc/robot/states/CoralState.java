package frc.robot.states;

import frc.robot.constants.CoralConstants;

public enum CoralState {
    NONE(0, 0),
    INTAKE(CoralConstants.kInchIntakeSpeed, 0),
    REVERSE(CoralConstants.kReverseSpeed, 0),
    INDEX(CoralConstants.kIndexSpeed, 0),
    READY(0, 0),
    ERROR(-1, 0),
    L1(CoralConstants.kL1Speed, CoralConstants.kSpeedDifference),
    L23(CoralConstants.kL23Speed, 0),
    L4(CoralConstants.kL4Speed, 0),
    INCH_INTAKE(CoralConstants.kInchIntakeSpeed, 0),
    INCH_REVERSE(-CoralConstants.kInchIntakeSpeed, 0);

    private double speed;
    private double speedDiff;

    CoralState(double speed, double speedDiff) {
        this.speed = speed;
        this.speedDiff = speedDiff;
    }

    public double getSpeed() {
        return speed;
    }

    public double getSpeedDiff() {
        return speedDiff;
    }
}
