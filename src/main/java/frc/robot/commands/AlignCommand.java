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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision;
import frc.robot.util.PARTsUnit;
import frc.robot.util.PARTsUnit.PARTsUnitType;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * Command that aligns the robot to the nearest detected AprilTag.
 * Interrupts if no AprilTag is found.
 */
public class AlignCommand extends Command {
  private final Vision m_Vision;
  private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> m_Swerve;
  private final SwerveRequest.FieldCentric m_alignRequest;

  private final PARTsUnit holdDistance;

  private final ProfiledPIDController aimController;
  private final ProfiledPIDController rangeController;

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
    
  private static final double RANGE_P = 4;
  private static final double RANGE_I = 0.04;
  private static final double RANGE_D = 0.1; //? ~10x P to prevent oscillation(?)  
  public AlignCommand(Vision vision, SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerve, PARTsUnit holdDistance) {
        m_Vision = vision;
        m_Swerve = swerve;
        this.holdDistance = holdDistance;
        
        this.m_alignRequest = new SwerveRequest.FieldCentric().withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1).withRotationalDeadband(0.1);

        aimController = new ProfiledPIDController(AIM_P, AIM_I, AIM_D, new TrapezoidProfile.Constraints(MAX_AIM_VELOCITY, MAX_AIM_ACCELERATION));
        aimController.enableContinuousInput(-Math.PI, Math.PI); //Wrpa from -pi to ip
        
        rangeController = new ProfiledPIDController(RANGE_P, RANGE_I, RANGE_D, new TrapezoidProfile.Constraints(MAX_RANGE_VELOCITY, MAX_RANGE_ACCELERATION));

        addRequirements(m_Vision);
    }

    @Override
    public void initialize() {
      // If we do not see a target, end the command.
      if (m_Vision.isTarget() != true) end(true);

      // This is not needed right now, remove it if you feel we do not need it ever.
      m_Vision.setPipelineIndex(0);
      
      // Initialize the aim controller.
      aimController.reset(m_Vision.getTX().to(PARTsUnitType.Radian));
      aimController.setGoal(0); // tx=0 is centered.
      aimController.setTolerance(0.1);


      // Initialize the range controller.
      rangeController.reset(m_Vision.getDistance(Constants.VisionConstants.REEF_APRILTAG_HEIGHT).getValue());
      rangeController.setGoal(holdDistance.getValue());
      rangeController.setTolerance(0.1);
    }

    @Override
    public void execute() {
      if (m_Vision.isTarget() != true) end(true);
      PARTsUnit currentDistance = m_Vision.getDistance(VisionConstants.REEF_APRILTAG_HEIGHT);
      PARTsUnit currentAngle = m_Vision.getTX();

      // TODO: Test minimum rotating value for aimController.
      double rotationOutput = aimController.calculate(currentAngle.to(PARTsUnitType.Radian));
      double rangeOutput = rangeController.calculate(currentDistance.getValue(), holdDistance.getValue());

      // Zero range output for testing.
      //rangeOutput = 0;
      // Zero rotation output for testing.
      //rotationOutput = 0;

      Translation2d translation = new Translation2d(rangeOutput, 0);

      System.out.println("AIM MEASURES:\nCurrent Angle " + currentAngle.getValue());
      System.out.println("Rotation Output: " + rotationOutput + "\n");
      //System.out.println("Aim Controller: " + aimController.getSetpoint().position);

      System.out.println("RANGEM MEASURES:\nCurrent Distance " + currentDistance.getValue());
      System.out.println("Range Output: " + rangeOutput + "\n");
      //System.out.println("Range Controller: " + rangeController.getSetpoint().position);
                  
      m_Swerve.setControl(m_alignRequest
          .withVelocityX(translation.getX())
          .withVelocityY(translation.getY())
          .withRotationalRate(rotationOutput));
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
      // TODO: Change to && once both the rangeController and the aimController work together.
      return aimController.atGoal() && rangeController.atGoal();
    }
}
