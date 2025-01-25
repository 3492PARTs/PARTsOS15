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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignCommand extends Command {
   // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= Drivetrain.kMaxAngularSpeed;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional(double dist)
  {    
    double kP = .1;
    double targetingForwardSpeed = dist * kP;
    targetingForwardSpeed *= Drivetrain.kMaxSpeed;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  private final Vision m_Vision;
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> m_Swerve;
    private final Distance holdDistance;
    private final SwerveRequest.RobotCentric m_alignRequest;

    private final ProfiledPIDController aimController;
    //private final ProfiledPIDController rangeController;

    //From tunerconsts and robtocontainer.java
    private static final double MAX_AIM_VELOCITY = 1.5*Math.PI; // radd/s
    private static final double MAX_AIM_ACCELERATION = Math.PI / 2; // rad/s^2
    private static final double MAX_RANGE_VELOCITY = 1.0; // m/s
    private static final double MAX_RANGE_ACCELERATION = 0.5; // m/2^s
  
    // Todo - Tune later
    private static final double AIM_P = 0.08; //Proprotinal
    private static final double AIM_I = 0; //0.01; //Gradual corretction
    private static final double AIM_D = 0; //0.05; //Smooth oscilattions
    
    private static final double RANGE_P = 0.1;
    private static final double RANGE_I = 0.01;
    private static final double RANGE_D = 0.05;

  public AlignCommand(Vision vision, SwerveDrivetrain<TalonFX, TalonFX, CANcoder> swerve, Distance holdDistance) {
        m_Vision = vision;
        m_Swerve = swerve;
        this.holdDistance = holdDistance;
        
        this.m_alignRequest = new SwerveRequest.RobotCentric().withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1).withRotationalDeadband(0.1);

        aimController = new ProfiledPIDController(AIM_P, AIM_I, AIM_D, new TrapezoidProfile.Constraints(MAX_AIM_VELOCITY, MAX_AIM_ACCELERATION));

        aimController.enableContinuousInput(-Math.PI, Math.PI); //Wrpa from -pi to ip
        
        //rangeController = new ProfiledPIDController(RANGE_P, RANGE_I, RANGE_D, new TrapezoidProfile.Constraints(MAX_RANGE_VELOCITY, MAX_RANGE_ACCELERATION));

        addRequirements(m_Vision);
    }

    @Override
    public void initialize() {
        //m_Vision.setPipelineIndex(0);
        
        aimController.reset(0);
        //rangeController.reset(m_Vision.getDistance(VisionConstants.REEF_APRILTAG_HEIGHT.in(Inches)).in(Inches)); //Init dist
        
        aimController.setGoal(0); // tx=0 is centered
        //rangeController.setGoal(holdDistance.in(Meters));
    }

    @Override
    public void execute() {
        //Angle tx = m_Vision.getTX();
        Distance currentDistance = m_Vision.getDistance(VisionConstants.REEF_APRILTAG_HEIGHT.in(Inches));

        double rotationOutput = limelight_aim_proportional(); // HELP MEEEEE //aimController.calculate(tx.in(Radians));
        //? This by itself causes no movement.
        double rangeOutput =  limelight_range_proportional(currentDistance.in(Meters)); //rangeController.calculate(currentDistance.in(Meters));

        Translation2d translation = new Translation2d(rangeOutput, 0);
                
        m_Swerve.setControl(m_alignRequest
            .withVelocityX(translation.getX())
            .withVelocityY(0) // Intentionally zero here.
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
        return aimController.atGoal(); //&& rangeController.atGoal();
    }

    public AlignCommand withTolerance(Angle aimTolerance, Distance rangeTolerance) {
        aimController.setTolerance(aimTolerance.in(Radians));
        //rangeController.setTolerance(rangeTolerance.in(Meters));
        return this;
    }
}
