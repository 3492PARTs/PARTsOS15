// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Candle;

/** Add your docs here. */
public class ElevatorSim extends Elevator {

    private final edu.wpi.first.wpilibj.simulation.ElevatorSim sim;

    private final String bottomLimitTopic = "Sim Controls/Bottom Limit";
    private final String topLimitTopic = "Sim Controls/Top Limit";

    public ElevatorSim(Candle candle) {
        super(candle);

        sim = new edu.wpi.first.wpilibj.simulation.ElevatorSim(
                DCMotor.getNEO(2),
                ElevatorConstants.gearRatio,
                ElevatorConstants.MASS_KG,
                ElevatorConstants.DRUM_RADIUS_METERS,
                ElevatorConstants.MIN_HEIGHT_METERS,
                ElevatorConstants.MAX_HEIGHT_METERS,
                false,
                ElevatorConstants.MIN_HEIGHT_METERS);

        sim.setState(ElevatorConstants.MIN_HEIGHT_METERS, 0);

        super.partsNT.putBoolean(bottomLimitTopic, false);
        super.partsNT.putBoolean(topLimitTopic, false);
    }

    @Override
    public double getElevatorPosition() {
        return sim == null ? 0 : sim.getPositionMeters();
    }

    @Override
    public double getRPS() {
        return sim.getVelocityMetersPerSecond() * 60 / ElevatorConstants.gearRatio; // 16 is the gear reduction

    }

    @Override
    protected void setSpeedWithoutLimits(double speed) {
        sim.setInput(speed);
    }

    @Override
    protected void setSpeedVoltageLimits(double voltage) {
        sim.setInputVoltage(voltage);
    }

    @Override
    protected void resetEncoder() {
        sim.setState(ElevatorConstants.MIN_HEIGHT_METERS, 0);
    }

    @Override
    public boolean getBottomLimit() {
        return super.partsNT.getBoolean(bottomLimitTopic);
    }

    @Override
    public boolean getTopLimit() {
        return super.partsNT.getBoolean(topLimitTopic);
    }
}
