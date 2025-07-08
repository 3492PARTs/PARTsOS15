package frc.robot.states;

import frc.robot.constants.ElevatorConstants;

public enum ElevatorState {
    MANUAL(-1),
    STOP(-1),
    STOW(ElevatorConstants.StowHeight),
    L2(ElevatorConstants.L2Height),
    L3(ElevatorConstants.L3Height),
    L4(ElevatorConstants.L4Height),
    A1(ElevatorConstants.LowAlgaeHeight),
    A2(ElevatorConstants.HighAlgaeHeight);

    private final double height;
    private double power = 0;

    ElevatorState(double height) {
        this.height = height;
    }

    public boolean hasTarget() {
        return height >= 0;
    }

    public double getTarget() {
        return height;
    }

    public double getPower() {
        return power;
    }

    public void setPower(double power) throws Exception {
        if (this != ElevatorState.MANUAL)
            throw new Exception("Can only set power on manual state");
        this.power = power;
    }
}
