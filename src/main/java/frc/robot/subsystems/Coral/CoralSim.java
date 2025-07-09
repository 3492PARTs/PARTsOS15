// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Coral;

import au.grapplerobotics.LaserCan;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Elevator.Elevator;

/** Add your docs here. */
public class CoralSim extends Coral {

    private final String entryTopic = "Sim Controls/In Entry";
    private final String exitTopic = "Sim Controls/In Exit";

    public CoralSim(Candle candle, Elevator elevator) {
        super(candle, elevator);
        super.partsNT.putBoolean(entryTopic, false);
        super.partsNT.putBoolean(exitTopic, false);
    }

    @Override
    public void periodic() {
        super.periodic();
        entryLaserMeasurement = generateLaserCanMeasurement(super.partsNT.getBoolean(entryTopic) ? 0 : 50);
        exitLaserMeasurement = generateLaserCanMeasurement(super.partsNT.getBoolean(exitTopic) ? 0 : 50);
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();
        super.partsNT.putDouble("Current/Left", coralState.getSpeed() - coralState.getSpeedDiff());
        super.partsNT.putDouble("Current/Right", -coralState.getSpeed());

        super.partsNT.putDouble("Output/Left", coralState.getSpeed() - coralState.getSpeedDiff());
        super.partsNT.putDouble("Output/Right", -coralState.getSpeed());
    }

    private LaserCan.Measurement generateLaserCanMeasurement(int measurement) {
        return new LaserCan.Measurement(0, measurement, 0, true, 0, new LaserCan.RegionOfInterest(0, 0, 0, 0));
    }

}
