// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.ElevatorConstants;

/** Add your docs here. */
public class ElevatorSim extends Elevator {

    private final edu.wpi.first.wpilibj.simulation.ElevatorSim sim;

    private final String bottomLimitTopic = "Sim Controls/Bottom Limit";
    private final String topLimitTopic = "Sim Controls/Top Limit";

    public ElevatorSim() {
        super();

        sim = new edu.wpi.first.wpilibj.simulation.ElevatorSim(
                DCMotor.getNEO(2),
                ElevatorConstants.Encoders.GEAR_RATIO,
                ElevatorConstants.MASS_KG,
                ElevatorConstants.DRUM_RADIUS_METERS,
                ElevatorConstants.MIN_HEIGHT_METERS,
                ElevatorConstants.MAX_HEIGHT_METERS,
                false,
                ElevatorConstants.MIN_HEIGHT_METERS);

        sim.setState(ElevatorConstants.MIN_HEIGHT_METERS, 0);

        super.partsNT.putBoolean(bottomLimitTopic, false);
        super.partsNT.putBoolean(topLimitTopic, false);

        carriageLaserCan = new LaserCan.Measurement(2, 0, 0, true, 0,
                new LaserCan.RegionOfInterest(0, 0, 0, 0));

        // Elevator Feedforward
        mElevatorFeedForward = new ElevatorFeedforward(
                0,
                0,
                0,
                0);
    }

    @Override
    public double getElevatorPosition() {
        return sim == null ? 0 : sim.getPositionMeters() * 8;
    }

    @Override
    public double getRPS() {
        return sim.getVelocityMetersPerSecond() * 60 / ElevatorConstants.GEAR_RATIO; // 16 is the gear reduction

    }

    @Override
    protected void setSpeedWithoutLimits(double speed) {
        sim.setInput(speed * RobotController.getBatteryVoltage());
        sim.update(0.02);
    }

    @Override
    protected void setSpeedVoltageLimits(double voltage) {
        sim.setInputVoltage(voltage);
        sim.update(0.02);
    }

    @Override
    protected void resetEncoder() {
        sim.setState(ElevatorConstants.MIN_HEIGHT_METERS, 0);
    }

    @Override
    public boolean getBottomLimit() {
        return super.partsNT.getBoolean(bottomLimitTopic) || getElevatorPosition() <= ElevatorConstants.STOW_HEIGHT;
    }

    @Override
    public boolean getTopLimit() {
        return super.partsNT.getBoolean(topLimitTopic) || getElevatorPosition() >= ElevatorConstants.MAX_HEIGHT;
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();

        super.partsNT.putDouble("Current/Left", sim.getCurrentDrawAmps());
        super.partsNT.putDouble("Current/Right", sim.getCurrentDrawAmps());

        super.partsNT.putDouble("Output/Left", sim.getOutput().mean());
        super.partsNT.putDouble("Output/Right", sim.getOutput().mean());
    }
}
