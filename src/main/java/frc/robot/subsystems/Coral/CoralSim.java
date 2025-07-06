// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Coral;

import au.grapplerobotics.LaserCan;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Elevator.Elevator;

/** Add your docs here. */
public class CoralSim extends Coral {

    public CoralSim(Candle candle, Elevator elevator) {
        super(candle, elevator);
        super.partsNT.putBoolean("In Entry", false);
        super.partsNT.putBoolean("In Exit", false);
    }

    @Override
    public void periodic() {
        super.periodic();
        mPeriodicIO.entryLaserMeasurement = generateLaserCanMeasurement(super.partsNT.getBoolean("In Entry") ? 0 : 50);
        mPeriodicIO.exitLaserMeasurement = generateLaserCanMeasurement(super.partsNT.getBoolean("In Exit") ? 0 : 50);
    }

    private LaserCan.Measurement generateLaserCanMeasurement(int measurement) {
        return new LaserCan.Measurement(0, measurement, 0, true, 0, new LaserCan.RegionOfInterest(0, 0, 0, 0));
    }

}
