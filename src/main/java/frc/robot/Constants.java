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
        public static final double kMaxSpeed = 3.0; // Meters per second.
    }
    public static class VisionConstants {
    //public static final String LIMELIGHT_NAME = "limelight-a";
    public static final Distance LIMELIGHT_LENS_HEIGHT = Distance.ofBaseUnits(8, Inches);
    public static final Angle LIMELIGHT_ANGLE = Angle.ofBaseUnits(0, Degrees);

    public static final Distance REEF_APRILTAG_HEIGHT = Distance.ofBaseUnits(6.875, Inches);
    public static final Distance PROCCESSOR_APRILTAG_HEIGHT = Distance.ofBaseUnits(45.875, Inches);
    public static final Distance CORAL_APRILTAG_HEIGHT = Distance.ofBaseUnits(53.25, Inches);
  }
  public static class Sensors {
    public static final int laserCanId = 30;
    public static final int canAndColorId = 31;
  }
  public static class Elevator {
    public static final int leftElevatorId = 100;
    public static final int rightElevatorId = 99;
  }
  public static class Algae {
    public static final int algaeIntakeId = 50;
    public static final int algaeWristId = 51;
    public static final double algaeIntakeSpeed = 0.2;
    public static final double algaeWristSpeed = 0.2;
  }
  public static class Coral {
    public static final int coralRightMotorId = 70;
    public static final int coralLeftMotorId = 71;
    public static final double coralMotorSpeed = 0.2;
  }
  public static class LED {
    public static final int LED_PORT = 1000;
    public static final int LED_LENGTH = 1001;
  }
}
