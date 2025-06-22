// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Vector;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.util.PARTsUnit;
import frc.robot.util.PARTsUnit.PARTsUnitType;

/**
 * Constants class to hold constants.
 */
public final class Constants {
  public static class Drivetrain {
    public static final String canBusName = "hi";
    public static final double kMaxAngularSpeed = Math.PI / 4; // Radians
    public static final double kMaxSpeed = .5; // Meters per second

    public static final double MAX_AIM_VELOCITY = 1.5 * Math.PI; // radd/s
    public static final double MAX_AIM_ACCELERATION = Math.PI / 2; // rad/s^2
    public static final double MAX_RANGE_VELOCITY = 2;//7.0;//5.0; // m/s
    public static final double MAX_RANGE_ACCELERATION = 2;//7.0;//5.0; // 0.5; // m/2^s

    public static final double THETA_P = 8.0; // Proprotinal
    public static final double THETA_I = 0.01; // 0.01; //Gradual corretction
    public static final double THETA_D = 0.05; // 0.05; //Smooth oscilattions

    public static final double RANGE_X_P = 2;//1.6;// 0.8; //12.5
    public static final double RANGE_I = 0; // 0.04 
    public static final double RANGE_D = 0; // 0.1

    public static final double RANGE_Y_P = .1; // 15.5

    public static final PARTsUnit yRControllerTolerance = new PARTsUnit(1, PARTsUnitType.Inch);
    public static final PARTsUnit xRControllerTolerance = new PARTsUnit(1.7, PARTsUnitType.Inch);
    public static final PARTsUnit thetaControllerTolerance = new PARTsUnit(2, PARTsUnitType.Angle);

    public static final double leftSideOffset = 5.5;

    //more positive, more to left
    public static final PARTsUnit leftAlignDistance = new PARTsUnit(6, PARTsUnitType.Inch);

    //more negative, more to right
    public static final PARTsUnit rightAlignDistance = new PARTsUnit(-6, PARTsUnitType.Inch); //-7.5

    public static final PARTsUnit L4XDistance = new PARTsUnit(-9 + (-4), PARTsUnitType.Inch);

    public static final PARTsUnit xZeroHoldDistance = new PARTsUnit(-9, PARTsUnitType.Inch);

    public static final double maxAlignTime = 4; //seconds
  }

  /*public static class LimelightData {
    String limelightName;
    PARTsUnit lensHeight;
    PARTsUnit limelightAngle;
  
    LimelightData(String limelightName, PARTsUnit lensHeight, PARTsUnit limelightAngle) {
      this.limelightName = limelightName;
      this.lensHeight = lensHeight;
      this.limelightAngle = limelightAngle;
    }
  }*/

  public static class VisionConstants {
    public static final String DRIVETRAIN_LIMELIGHT = "limelight-slimmy"; //The_Real
    public static final String ELEVATOR_LIMELIGHT = "limelight-thereal";
    public static final PARTsUnit LIMELIGHT_LENS_HEIGHT = new PARTsUnit(9, PARTsUnitType.Inch); // Inches
    public static final PARTsUnit LIMELIGHT_ANGLE = new PARTsUnit(0, PARTsUnitType.Angle); // Degrees

    public static final double REEF_APRILTAG_HEIGHT = 16; //Distance.ofBaseUnits(6.875, Inches);
    public static final double PROCCESSOR_APRILTAG_HEIGHT = 45.875; // Inches
    public static final double CORAL_APRILTAG_HEIGHT = 53.25; // Inches

     public static final edu.wpi.first.math.Vector<N3> MT1_STDEVS = VecBuilder.fill(0.5, 0.5, 1.0);
     public static final edu.wpi.first.math.Vector<N3> MT2_STDEVS = VecBuilder.fill(0.7, 0.7, 3492);
  }

  public static class Elevator {
    public static final int leftElevatorId = 2;
    public static final int rightElevatorId = 1;
    public static final int laserCanId = 32;
    public static final int StowHeight = 0;
    public static final double L2Height = 10.6;//8.785;
    public static final double L3Height = 34.1;//31.9;
    public static final double L4Height = 74;//73.38;//73.16;//73.80; //best so far 74.07373046875, 73.59748840332031
    public static final double LowAlgaeHeight = 33.00;//29.47;
    public static final double HighAlgaeHeight = 58;//55.548;
    public static final int L_SWITCH_PORT = 0;
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kTolerance = 0.25;
    public static final int kIZone = 0;
    public static final int kMaxCurrent = 40;
    public static final int kMaxVelocity = 100;
    public static final int kMaxAcceleration = 300;
    public static final double kG = 0.18162;//0.3766;
    public static final double kS = 0.22467;//0.24105;
    public static final double kV = 0.020157;//0.020009;
    public static final double kA = 0.0022691;//0.0022395;
    public static final double maxSpeed = 0.5;
    public static final double maxHeight = 74.0;
    public static final double maxLaserCanHeight = 27;//40;
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

    public static final double kWristP = 10.0;
    public static final double kWristI = 0.0;
    public static final double kWristD = 0.0;
    public static final double kTolerance = new PARTsUnit(1, PARTsUnitType.Angle).to(PARTsUnitType.Radian);

    public static final double kWristKS = 0.25008;
    public static final double kWristKG = 0.10667;
    public static final double kWristKV = 0.0019276;
    public static final double kWristKA = 0.00032922;

    public static final double kWristOffset = 0;//141.0;

    public static final double kWristMaxVelocity = 690.0;
    public static final double kWristMaxAcceleration = 1380.0;

    public static final double kStowAngle = 0;
    public static final double kDeAlgaeAngle = 139.15;
    public static final double kGroundIntakeAngle = 145.0;

    public static final int wristGearRatio = 64 / 1;

    // INTAKE
    public static final int kMaxIntakeCurrent = 20;

    public static final double kReefIntakeSpeed = 0.8;
    public static final double kEjectSpeed = 0.3;
    public static final double kGroundIntakeSpeed = -0.3;

    public static final Pose2d deAlgaeDistance = new Pose2d(0.5, 0, new Rotation2d());

  }

  public static class Coral {
    public static final int coralRightMotorId = 5;
    public static final int coralLeftMotorId = 6;
    public static final int laserCanId = 30;
    public static final int laserCan2Id = 31;

    public static final double kIntakeSpeed = -0.15;//-0.3;
    public static final double kReverseSpeed = 0.6;
    public static final double kL1Speed = -0.22;
    public static final double kL23Speed = -0.25;
    public static final double kL4Speed = -.2;
    public static final double kIndexSpeed = -0.135;
    public static final double kSpeedDifference = kL1Speed * 0.9;
    public static final double kInchIntakeSpeed = -.1;
  }

  public static class Candle {
    public static final int candleId = 33;
    public static final int ledLength = 1000;
  }

  public static class Climber {
    public static final int climberId = 34;
  }

  public static class Debug {
    public static boolean logging = true;
    public static boolean debug = true;
    public static boolean allowAutoControllerDetection = false;
  }
}
