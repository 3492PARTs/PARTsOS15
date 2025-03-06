// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

import frc.robot.util.PARTsUnit;

/**
 * Constants class to hold constants.
 */
public final class Climber {
  public static class Drivetrain {
    public static final double kMaxAngularSpeed = Math.PI / 4; // Radians
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
    public static final int laserCanId = 32;
    public static final int StowHeight = 0;
    public static final double L2Height = 11.0;
    public static final double L3Height = 35.0;
    public static final double L4Height = 72.5;
    public static final double LowAlgaeHeight = 0;
    public static final double HighAlgaeHeight = 0;
    public static final int L_SWITCH_PORT = 0;
    public static final double kP = 0.3;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kTolerance = 0.5;
    public static final int kIZone = 0;
    public static final int kMaxCurrent = 40;
    public static final int kMaxVelocity = 60;
    public static final int kMaxAcceleration = 200;
    public static final double kG = 0.3766;
    public static final double kS = 0.24105;
    public static final double kV = 0.020009;
    public static final double kA = 0.0022395;
    public static final double maxSpeed = 0.5;
    public static final double maxHeight = 72.5;
    public static final double maxLaserCanHeight = 40;
    public static final double gearRatio = 16 / 1;
    public static final double homingSpeed = -0.1;
    public static final double bottomLimitPositionErrorMargin = 30;
  }

  public static class Algae {
    public static final int algaeIntakeId = 3;
    public static final int algaeWristId = 4;
    public static final double algaeIntakeSpeed = 0.2;
    public static final double maxWristSpeed = .5;
    public static final int kMaxWristCurrent = 10;

    public static final double kWristP = 5.0;
    public static final double kWristI = 0.0;
    public static final double kWristD = 0.0;
    public static final double kTolerance = 1.0;

    public static final double kWristKS = 0.25008;
    public static final double kWristKG = 0.10667;
    public static final double kWristKV = 0.0019276;
    public static final double kWristKA = 0.00032922;

    public static final double kWristOffset = 0;//141.0;

    public static final double kWristMaxVelocity = 690.0;
    public static final double kWristMaxAcceleration = 1380.0;

    public static final double kStowAngle = 0;
    public static final double kDeAlgaeAngle = 150.0;
    public static final double kGroundIntakeAngle = 145.0;

    public static final int wristGearRatio = 64 / 1;

    // INTAKE
    public static final int kMaxIntakeCurrent = 20;

    public static final double kReefIntakeSpeed = 0.6;
    public static final double kEjectSpeed = 0.3;
    public static final double kGroundIntakeSpeed = -0.3;
  }

  public static class Coral {
    public static final int coralRightMotorId = 5;
    public static final int coralLeftMotorId = 6;
    public static final int laserCanId = 30;
    public static final int canAndColorId = 31;

    public static final double kIntakeSpeed = -0.3;
    public static final double kReverseSpeed = 0.6;
    public static final double kL1Speed = -0.4;
    public static final double kL24Speed = -0.4;
    public static final double kIndexSpeed = -0.1;
    public static final double kSpeedDifference = kL1Speed * 0.5;
  }

  public static class Candle {
    public static final int candleId = 33;
    public static final int ledLength = 1000;
  }
  /*
  public static class Climber {
    public static final int climberId = 34;
  } 
    */

  public static class Debug {
    public static boolean logging = true;
    public static boolean debug = true;
  }
}
