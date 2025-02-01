// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.util.PARTsUnit;

/**
 * Constants class to hold constants.
 */
public final class Constants {

    public static class LimelightData {
      String limelightName;
      PARTsUnit lensHeight;
      PARTsUnit limelightAngle;

      LimelightData(String limelightName, PARTsUnit lensHeight, PARTsUnit limelightAngle) {
        this.limelightName = limelightName;
        this.lensHeight = lensHeight;
        this.limelightAngle = limelightAngle;
      }
    }
    public static class Drivetrain {
        public static final double kMaxAngularSpeed = Math.PI/4; // Radians
        public static final double kMaxSpeed = .5; // Meters per second.
    }
    public static class VisionConstants {
      public static final String DRIVETRAIN_LIMELIGHT = ""; //The_Real
      public static final String ELEVATOR_LIMELIGHT = "Slim_Shady";
      public static final double LIMELIGHT_LENS_HEIGHT = 8.75; // Inches
      public static final double LIMELIGHT_ANGLE = 15; // Degrees

      public static final double REEF_APRILTAG_HEIGHT = 16; //Distance.ofBaseUnits(6.875, Inches);
      public static final double PROCCESSOR_APRILTAG_HEIGHT = 45.875; // Inches
      public static final double CORAL_APRILTAG_HEIGHT = 53.25; // Inches
  }
}
