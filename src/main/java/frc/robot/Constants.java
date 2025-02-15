// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.PARTsUnit;

/**
 * Constants class to hold constants.
 */
public final class Constants {
    public static class Drivetrain {
      public static final double kMaxAngularSpeed = Math.PI/4; // Radians
      public static final double kMaxSpeed = .5; // Meters per seco
    }
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
    public static class VisionConstants {
      public static final String DRIVETRAIN_LIMELIGHT = ""; //The_Real
      public static final String ELEVATOR_LIMELIGHT = "Slim_Shady";
      public static final double LIMELIGHT_LENS_HEIGHT = 8.75; // Inches
      public static final double LIMELIGHT_ANGLE = 11; // Degrees

      public static final double REEF_APRILTAG_HEIGHT = 16; //Distance.ofBaseUnits(6.875, Inches);
      public static final double PROCCESSOR_APRILTAG_HEIGHT = 45.875; // Inches
      public static final double CORAL_APRILTAG_HEIGHT = 53.25; // Inches
  }

  public static class Elevator {
    public static final int leftElevatorId = 2;
    public static final int rightElevatorId = 1;
    public static final int StowHeight = 0;
    public static final int L2Height = 0;
    public static final int L3Height = 0;
    public static final int L4Height = 0;
    public static final int LowAlgaeHeight = 0;
    public static final int HighAlgaeHeight = 0;
    public static final int L_SWITCH_PORT = 0;
    public static final int kP = 0;
    public static final int kI = 0;
    public static final int kD = 0;
    public static final int kIZone = 0;
    public static final int kMaxCurrent = 0;
    public static final int kMaxVelocity = 1;
    public static final int kMaxAcceleration = 1;
    public static final int kG = 0;

  }
  public static class Algae {
    public static final int algaeIntakeId = 3;
    public static final int algaeWristId = 4;
    public static final double algaeIntakeSpeed = 0.2;
    public static final double maxWristSpeed = .5;
    public static final int kMaxWristCurrent = 10;

    public static final double kWristP = 0.01;
    public static final double kWristI = 0.0;
    public static final double kWristD = 0.0;

    public static final double kWristKS = 0.0;
    public static final double kWristKG = 0.0;
    public static final double kWristKV = 0.100;
    public static final double kWristKA = 0.0;

    public static final double kWristOffset = 141.0;

    public static final double kWristMaxVelocity = 690.0;
    public static final double kWristMaxAcceleration = 1380.0;

    public static final double kStowAngle = 233.0;
    public static final double kDeAlgaeAngle = 215.0;
    public static final double kGroundIntakeAngle = 162.0;

    // INTAKE
    public static final int kMaxIntakeCurrent = 20;

    public static final double kIntakeSpeed = 0.6;
    public static final double kEjectSpeed = -0.3;
    public static final double kGroundIntakeSpeed = -0.3;
  }
  
  public static class Coral {
    public static final int coralRightMotorId = 5;
    public static final int coralLeftMotorId = 6;
    public static final int laserCanId = 30;
    public static final int canAndColorId = 31;
    
    public static final double kIntakeSpeed = -0.3;
    public static final double kReverseSpeed = 0.3;
    public static final double kL1Speed = 0.4;
    public static final double kL24Speed = 0.4;
    public static final double kIndexSpeed = 0.1;
    public static final double kSpeedDifference = kL1Speed * 0.5;
  }
  public static class LED {
    public static final int LED_PORT = 1000;
    public static final int LED_LENGTH = 1001;
  }
  public enum ElevatorState {
    NONE,
    STOW,
    L2,
    L3,
    L4,
    A1,
    A2
  }
}
