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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * Command that aligns the robot to the nearest detected AprilTag.
 * Interrupts if no AprilTag is found.
 */
public class AlignCommand extends Command {
  private final Vision m_Vision;
  private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> m_Swerve;
  private final SwerveRequest.RobotCentric m_alignRequest;

  private final Pose2d holdDistance;

  private final ProfiledPIDController thetaController;
  private final ProfiledPIDController xRangeController;
  private final ProfiledPIDController yRangeController;

  /** 
   * From {@link frc.robot.generated.TunerConstants TunerConstants}
   * and {@link frc.robot.RobotContainer RobotContainer}
   */
  private static final double MAX_AIM_VELOCITY = 1.5*Math.PI; // radd/s
  private static final double MAX_AIM_ACCELERATION = Math.PI / 2; // rad/s^2
  private static final double MAX_RANGE_VELOCITY = 1.0; // m/s
  private static final double MAX_RANGE_ACCELERATION = 0.5; // m/2^s
  
  // Todo - Tune later
  private static final double AIM_P = 1.8; //Proprotinal
  private static final double AIM_I = 0.01; //0.01; //Gradual corretction
  private static final double AIM_D = 0.05; //0.05; //Smooth oscilattions
    
  private static final double RANGE_P = 0.8;
  private static final double RANGE_I = 0.04;
  private static final double RANGE_D = 0.1; //? ~10x P to prevent oscillation(?) 

  // Robot poses.
  private Pose3d initialRobotPose3d;
  private Pose3d currentRobotPose3d;

  DoubleLogEntry L_rangeOutput;
  DoubleLogEntry L_rotationOutput;
  DoubleLogEntry L_currentDistance;
  DoubleLogEntry L_currentAngle;
  DoubleLogEntry L_llposeX;
  DoubleLogEntry L_llposeY;
  DoubleLogEntry L_rposeX;
  DoubleLogEntry L_rposeY;

  public AlignCommand(Vision vision, SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerve, Pose2d holdDistance) {
        m_Vision = vision;
        m_Swerve = swerve;
        this.holdDistance = holdDistance;
        
        this.m_alignRequest = new SwerveRequest.RobotCentric().withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1).withRotationalDeadband(0.1);

        thetaController = new ProfiledPIDController(AIM_P, AIM_I, AIM_D, new TrapezoidProfile.Constraints(MAX_AIM_VELOCITY, MAX_AIM_ACCELERATION));
        thetaController.enableContinuousInput(-Math.PI, Math.PI); //Wrpa from -pi to ip
        
        xRangeController = new ProfiledPIDController(RANGE_P, RANGE_I, RANGE_D, new TrapezoidProfile.Constraints(MAX_RANGE_VELOCITY, MAX_RANGE_ACCELERATION));
        yRangeController = new ProfiledPIDController(RANGE_P, RANGE_I, RANGE_D, new TrapezoidProfile.Constraints(MAX_RANGE_VELOCITY, MAX_RANGE_ACCELERATION));

        // Set up custom log entries
        DataLog log = DataLogManager.getLog();
        L_currentDistance = new DoubleLogEntry(log, "/PARTs/align/currentDistance");
        L_currentAngle = new DoubleLogEntry(log, "/PARTs/align/currentAngle");
        L_rangeOutput = new DoubleLogEntry(log, "/PARTs/align/rangeOutput");
        L_rotationOutput = new DoubleLogEntry(log, "/PARTs/align/rotationOutput");
        L_llposeX = new DoubleLogEntry(log, "/PARTs/align/llposeX");
        L_llposeY = new DoubleLogEntry(log, "/PARTs/align/llposeY");
        L_rposeX = new DoubleLogEntry(log, "/PARTs/align/rposeX");
        L_rposeY = new DoubleLogEntry(log, "/PARTs/align/rposeY");


        addRequirements(m_Vision);
    }

    @Override
    public void initialize() {
      // If we do not see a target, end the command.
      if (m_Vision.isTarget() != true) end(true);

      // This is not needed right now, remove it if you feel we do not need it ever.
      m_Vision.setPipelineIndex(0);

      // Reset pose to zero.
      m_Swerve.resetPose(new Pose2d(0,0, null));
      
      // Get init. distance from camera.
      initialRobotPose3d = m_Vision.convertToKnownSpace(m_Vision.getPose3d());
      m_Swerve.resetPose(initialRobotPose3d.toPose2d());

      L_llposeX.append(initialRobotPose3d.getX());
      L_llposeY.append(initialRobotPose3d.getY());
      
      // Initialize the aim controller.
      thetaController.reset(initialRobotPose3d.getRotation().getAngle());
      thetaController.setGoal(holdDistance.getRotation().getRadians()); // tx=0 is centered.
      thetaController.setTolerance(0.1);

      // Initialize the x-range controller.
      xRangeController.reset(initialRobotPose3d.getX());
      xRangeController.setGoal(holdDistance.getX());
      xRangeController.setTolerance(0.1);

      // Initialize the y-range controller.
      yRangeController.reset(initialRobotPose3d.getY()); // Center to target.
      yRangeController.setGoal(holdDistance.getY()); // Center to target.
      yRangeController.setTolerance(0.1);
    }

    @Override
    public void execute() {
      currentRobotPose3d = new Pose3d(m_Swerve.getState().Pose);

      L_rposeX.append(currentRobotPose3d.getX());
      L_rposeY.append(currentRobotPose3d.getY());

      /* Math not needed(?)
      Pose2d newRobotPose2d = new Pose2d(
        -(holdDistance.getX() - currentRobotPose3d.getX()),
        -(holdDistance.getY() - currentRobotPose3d.getY()),
        currentRobotPose3d.getRotation().toRotation2d()
      );
      */

      Rotation2d rotationOutput = new Rotation2d(
        thetaController.calculate(currentRobotPose3d.getRotation().toRotation2d().getRadians()));

      Pose2d rangeOutput = new Pose2d(
        -xRangeController.calculate(currentRobotPose3d.getX(), holdDistance.getX()), 
        -yRangeController.calculate(currentRobotPose3d.getY(), holdDistance.getY()), 
        null);

      // Get dist. from drivetrain.

      Translation2d translation = new Translation2d(rangeOutput.getX(), rangeOutput.getY());

      //System.out.println("AIM MEASURES:\nCurrent Angle " + currentAngle.getValue());
      //System.out.println("Rotation Output: " + rotationOutput + "\n");
      //System.out.println("Aim Controller: " + aimController.getSetpoint().position);

      //L_currentAngle.append(currentAngle.getValue());
      //L_rotationOutput.append(rotationOutput);
      //L_currentDistance.append(currentDistance.getValue());
      //L_rangeOutput.append(rangeOutput);

      System.out.println("RANGE MEASURES:\nCurrent Distance: (" + currentRobotPose3d.getX() + ", " + currentRobotPose3d.getY() + ")");
      System.out.println("Range Output: (" + rangeOutput.getX() + ", " + rangeOutput.getY() + ")\n");
      //System.out.println("Range Controller: " + rangeController.getSetpoint().position);
                  
      m_Swerve.setControl(m_alignRequest
          .withVelocityX(translation.getX())
          .withVelocityY(translation.getY())
          .withRotationalRate(rotationOutput.getRadians()*0)
      );
    }
    
    @Override
    public void end(boolean interrupted) {
      m_Swerve.setControl(m_alignRequest
          .withVelocityX(0)
          .withVelocityY(0)
          .withRotationalRate(0));
    }

    @Override
    public boolean isFinished() {
      return xRangeController.atGoal() && yRangeController.atGoal();// && thetaController.atGoal();
    }
}
