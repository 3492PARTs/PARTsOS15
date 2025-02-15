// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.util.PARTsUnit;
import frc.robot.util.PARTsUnit.PARTsUnitType;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * Command that aligns the robot to the nearest detected AprilTag.
 * Interrupts if no AprilTag is found.
 */
public class AlignCommand extends Command {
  private boolean kill = false;

  private final Vision m_Vision;
  private final CommandSwerveDrivetrain m_drivetrain;
  private final SwerveRequest.RobotCentric m_alignRequest;

  private final Pose2d holdDistance;

  private final ProfiledPIDController thetaController;
  private final ProfiledPIDController xRangeController;
  private final ProfiledPIDController yRangeController;

  /**
   * From {@link frc.robot.generated.TunerConstants TunerConstants}
   * and {@link frc.robot.RobotContainer RobotContainer}
   */
  private static final double MAX_AIM_VELOCITY = 1.5 * Math.PI; // radd/s
  private static final double MAX_AIM_ACCELERATION = Math.PI / 2; // rad/s^2
  private static final double MAX_RANGE_VELOCITY = 1.0; // m/s
  private static final double MAX_RANGE_ACCELERATION = 1.5; // 0.5; // m/2^s

  // Todo - Tune later
  private static final double THETA_P = 1.8; // Proprotinal
  private static final double THETA_I = 0.01; // 0.01; //Gradual corretction
  private static final double THETA_D = 0.05; // 0.05; //Smooth oscilattions

  private static final double RANGE_P = 2.0;//1.6;// 0.8;
  private static final double RANGE_I = 0.04;
  private static final double RANGE_D = 0.1; // ? ~10x P to prevent oscillation(?)

  // Robot poses.
  private Pose3d initialRobotPose3d;
  private Pose3d currentRobotPose3d;

  DoubleLogEntry L_rangeOutputX;
  DoubleLogEntry L_rangeOutputY;
  DoubleLogEntry L_rotationOutput;
  DoubleLogEntry L_currentDistance;
  DoubleLogEntry L_currentAngle;
  DoubleLogEntry L_llposeX;
  DoubleLogEntry L_llposeY;
  DoubleLogEntry L_llposeRot;
  DoubleLogEntry L_rposeX;
  DoubleLogEntry L_rposeY;
  DoubleLogEntry L_rposeRot;
  BooleanLogEntry L_RangeGoalX;
  BooleanLogEntry L_RangeGoalY;
  BooleanLogEntry L_ThetaGoal;

  public AlignCommand(Vision vision, CommandSwerveDrivetrain swerve, Pose2d holdDistance) {
    m_Vision = vision;
    m_drivetrain = swerve;
    this.holdDistance = holdDistance;

    this.m_alignRequest = new SwerveRequest.RobotCentric()
        .withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1).withRotationalDeadband(0.1);

    thetaController = new ProfiledPIDController(THETA_P, THETA_I, THETA_D,
        new TrapezoidProfile.Constraints(MAX_AIM_VELOCITY, MAX_AIM_ACCELERATION));
    thetaController.enableContinuousInput(-Math.PI, Math.PI); // Wrpa from -pi to ip

    xRangeController = new ProfiledPIDController(RANGE_P, RANGE_I, RANGE_D,
        new TrapezoidProfile.Constraints(MAX_RANGE_VELOCITY, MAX_RANGE_ACCELERATION));
    yRangeController = new ProfiledPIDController(RANGE_P, RANGE_I, RANGE_D,
        new TrapezoidProfile.Constraints(MAX_RANGE_VELOCITY, MAX_RANGE_ACCELERATION));

    // Set up custom log entries
    DataLog log = DataLogManager.getLog();

    L_currentDistance = new DoubleLogEntry(log, "/PARTs/align/currentDistance");
    L_currentAngle = new DoubleLogEntry(log, "/PARTs/align/currentAngle");
    L_rangeOutputX = new DoubleLogEntry(log, "/PARTs/align/rangeOutputX");
    L_rangeOutputY = new DoubleLogEntry(log, "/PARTs/align/rangeOutputY");
    L_rotationOutput = new DoubleLogEntry(log, "/PARTs/align/rotationOutput");

    L_llposeX = new DoubleLogEntry(log, "/PARTs/align/llposeX");
    L_llposeY = new DoubleLogEntry(log, "/PARTs/align/llposeY");
    L_llposeRot = new DoubleLogEntry(log, "/PARTs/align/llposeRot");

    L_rposeX = new DoubleLogEntry(log, "/PARTs/align/rposeX");
    L_rposeY = new DoubleLogEntry(log, "/PARTs/align/rposeY");
    L_rposeRot = new DoubleLogEntry(log, "/PARTs/align/rposeRot");

    L_RangeGoalX = new BooleanLogEntry(log, "/PARTs/align/rangeGoalX");
    L_RangeGoalY = new BooleanLogEntry(log, "/PARTs/align/rangeGoalY");
    L_ThetaGoal = new BooleanLogEntry(log, "/PARTs/align/thetaGoal");

    addRequirements(m_Vision);
  }

  @Override
  public void initialize() {
    // If we do not see a target, end the command.
    if (m_Vision.isTarget() != true)
      end(true);

    // This is not needed right now, remove it if you feel we do not need it ever.
    m_Vision.setPipelineIndex(0);

    // Reset pose to zero.
    // TODO: See about removal
    // m_drivetrain.resetPose(new Pose2d(0,0, new Rotation2d(0,0)));

    m_drivetrain.updatePoseEstimator();
    // Get init. distance from camera.
    Rotation2d estRot = m_drivetrain.getEstimatedRotation2d();

    if (estRot == null) {
      System.out.println("mt2 is null");
      end(true);
      kill = true;
    } else {
      Rotation3d rotation = new Rotation3d(estRot);

      initialRobotPose3d = m_Vision.convertToKnownSpace(m_Vision.getPose3d(), rotation);

      m_drivetrain.resetPose(initialRobotPose3d.toPose2d());

      L_llposeX.append(initialRobotPose3d.getX());
      L_llposeY.append(initialRobotPose3d.getY());
      L_llposeRot.append(initialRobotPose3d.getRotation().getAngle());

      // Initialize the aim controller.
      thetaController.reset(
          new PARTsUnit(initialRobotPose3d.getRotation().getAngle(), PARTsUnitType.Angle).to(PARTsUnitType.Radian));
      thetaController.setGoal(holdDistance.getRotation().getRadians()); // tx=0 is centered.
      thetaController.setTolerance(new PARTsUnit(2, PARTsUnitType.Angle).to(PARTsUnitType.Radian));

      // Initialize the x-range controller.
      xRangeController.reset(initialRobotPose3d.getX());
      xRangeController.setGoal(holdDistance.getX());
      xRangeController.setTolerance(0.1);

      // Initialize the y-range controller.
      yRangeController.reset(initialRobotPose3d.getY()); // Center to target.
      yRangeController.setGoal(holdDistance.getY()); // Center to target.
      yRangeController.setTolerance(0.1);
    }
  }

  @Override
  public void execute() {
    currentRobotPose3d = new Pose3d(m_drivetrain.getState().Pose);

    L_rposeX.append(currentRobotPose3d.getX());
    L_rposeY.append(currentRobotPose3d.getY());
    L_rposeRot.append(currentRobotPose3d.getRotation().getAngle());

    /*
     * Math not needed(?)
     * Pose2d newRobotPose2d = new Pose2d(
     * -(holdDistance.getX() - currentRobotPose3d.getX()),
     * -(holdDistance.getY() - currentRobotPose3d.getY()),
     * currentRobotPose3d.getRotation().toRotation2d()
     * );
     */

    Rotation2d rotationOutput = new Rotation2d(
        thetaController.calculate(currentRobotPose3d.getRotation().toRotation2d().getRadians()));

    Pose2d rangeOutput = new Pose2d(
        xRangeController.calculate(currentRobotPose3d.getX(), holdDistance.getX()),
        yRangeController.calculate(currentRobotPose3d.getY(), holdDistance.getY()),
        null);

    // Get dist. from drivetrain.

    Translation2d translation = new Translation2d(rangeOutput.getX(), rangeOutput.getY());

    // System.out.println("AIM MEASURES:\nCurrent Angle " +
    // currentRobotPose3d.getRotation().toRotation2d().getDegrees());
    // System.out.println("Rotation Output: " + rotationOutput.getDegrees() + "\n");
    // System.out.println("Aim Controller: " +
    // thetaController.getSetpoint().position);

    L_rotationOutput.append(rotationOutput.getDegrees());
    L_rangeOutputX.append(rangeOutput.getX());
    L_rangeOutputY.append(rangeOutput.getY());

    L_RangeGoalX.append(xRangeController.atGoal());
    L_RangeGoalY.append(yRangeController.atGoal());
    L_ThetaGoal.append(thetaController.atGoal());

    // System.out.println("RANGE MEASURES:\nCurrent Distance: (" +
    // currentRobotPose3d.getX() + ", " + currentRobotPose3d.getY() + ")");
    // System.out.println("Range Output: (" + rangeOutput.getX() + ", " +
    // rangeOutput.getY() + ")\n");

    System.out.println("rotation angle: " + currentRobotPose3d.getRotation().getAngle());
    m_drivetrain.updatePoseEstimator();
    // Get init. distance from camera.
    Rotation2d estRot = m_drivetrain.getEstimatedRotation2d();
    System.out.println("ll angle: " + estRot.getDegrees());

    m_drivetrain.setControl(m_alignRequest
        .withVelocityX(translation.getX())
        .withVelocityY(translation.getY())
        .withRotationalRate(rotationOutput.getRadians()));
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setControl(m_alignRequest
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0));
  }

  @Override
  public boolean isFinished() {
    return (xRangeController.atGoal() && yRangeController.atGoal() && thetaController.atGoal()) || kill;
  }
}
