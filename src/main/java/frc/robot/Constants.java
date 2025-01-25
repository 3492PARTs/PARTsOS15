// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/**
 * Constants class to hold constants.
 */
public final class Constants {
    public static class Drivetrain {
        public static final double kMaxAngularSpeed = Math.PI/4; // Radians
        public static final double kMaxSpeed = .5; // Meters per second.
    }
    public static class VisionConstants {
    //public static final String LIMELIGHT_NAME = "limelight-a";
    public static final Distance LIMELIGHT_LENS_HEIGHT = Distance.ofBaseUnits(8.75, Inches);
    public static final Angle LIMELIGHT_ANGLE = Angle.ofBaseUnits(15, Degrees);

    public static final Distance REEF_APRILTAG_HEIGHT = Distance.ofBaseUnits(16, Inches); //Distance.ofBaseUnits(6.875, Inches);
    public static final Distance PROCCESSOR_APRILTAG_HEIGHT = Distance.ofBaseUnits(45.875, Inches);
    public static final Distance CORAL_APRILTAG_HEIGHT = Distance.ofBaseUnits(53.25, Inches);
  }
}
