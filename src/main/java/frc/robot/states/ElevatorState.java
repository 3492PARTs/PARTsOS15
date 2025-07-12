package frc.robot.states;

import frc.robot.constants.ElevatorConstants;

public enum ElevatorState {
    MANUAL(-1),
    STOP(-1),
    STOW(ElevatorConstants.STOW_HEIGHT),
    L2(ElevatorConstants.L2_HEIGHT),
    L3(ElevatorConstants.L3_HEIGHT),
    L4(ElevatorConstants.L4_HEIGHT),
    A1(ElevatorConstants.LOW_ALGAE_HEIGHT),
    A2(ElevatorConstants.HIGH_ALGAE_HEIGHT);

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
