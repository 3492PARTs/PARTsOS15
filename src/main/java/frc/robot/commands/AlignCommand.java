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

  private final double holdDistance;

  private final ProfiledPIDController aimController;
  private final ProfiledPIDController rangeController;

  //From tunerconsts and robtocontainer.java
  private static final double MAX_AIM_VELOCITY = 1.5*Math.PI; // radd/s
  private static final double MAX_AIM_ACCELERATION = Math.PI / 2; // rad/s^2
  private static final double MAX_RANGE_VELOCITY = 1.0; // m/s
  private static final double MAX_RANGE_ACCELERATION = 0.5; // m/2^s
  
  // Todo - Tune later
  private static final double AIM_P = 0.4; //Proprotinal
  private static final double AIM_I = 0.01; //0.01; //Gradual corretction
  private static final double AIM_D = 0.05; //0.05; //Smooth oscilattions
    
  private static final double RANGE_P = 1.4;
  private static final double RANGE_I = 0.01;
  private static final double RANGE_D = 0.05; //? ~10x P to prevent oscillation(?)  
  public AlignCommand(Vision vision, SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerve, double holdDistance) {
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
      // If we do not see a target, exit the command.
      if (m_Vision.isTarget() != true) end(true);

      // This is not needed right now, remove it if you feel we do not need it ever.
      m_Vision.setPipelineIndex(0);
      
      // Init the aim controller.
      aimController.reset(new PARTsUnit(m_Vision.getTX(), PARTsUnitType.Angle).to(PARTsUnitType.Radian));
      aimController.setGoal(0); // tx=0 is centered.
      aimController.setTolerance(0.1);

      // Init the range controller.
      rangeController.reset(m_Vision.getDistance(0)); //Init dist
      rangeController.setGoal(holdDistance);
      rangeController.setTolerance(0.1);
    }

    @Override
    public void execute() {
      double currentDistance = m_Vision.getDistance(VisionConstants.REEF_APRILTAG_HEIGHT);
      double currentAngle = m_Vision.getTX();

      // TODO: Test minimum rotating value for aimController.
      double rotationOutput = aimController.calculate(PARTsUnit.DegreesToRadians.apply(currentAngle));
      double rangeOutput = rangeController.calculate(currentDistance, holdDistance);

      // Zero range output for testing.
      rangeOutput = 0;

      Translation2d translation = new Translation2d(rangeOutput, 0);

      System.out.println("Current Angle " + currentAngle);
      System.out.println("Rotation Output: " + rotationOutput);
      System.out.println("Controller: " + aimController.getSetpoint().position);
                  
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
      return aimController.atGoal() || rangeController.atGoal();
    }
}
