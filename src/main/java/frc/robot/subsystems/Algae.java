// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.PARTsUnit;
import frc.robot.util.PARTsUnit.PARTsUnitType;

public class Algae extends PARTsSubsystem {
  
  /*-------------------------------- Private instance variables ---------------------------------*/
  private PeriodicIO mPeriodicIO;

  public enum IntakeState {
    NONE,
    STOW,
    DEALGAE,
    GROUND
  }

  private SparkMax mWristMotor;
  private final ProfiledPIDController mWristPIDController;
  private final ArmFeedforward mWristFeedForward;

  private SparkMax mIntakeMotor;

  private final RelativeEncoder mWristAbsEncoder;

  public Algae() {
    super("Algae");

    mPeriodicIO = new PeriodicIO();

    // WRIST
    mWristMotor = new SparkMax(Constants.Algae.algaeWristId, MotorType.kBrushless);
    mWristAbsEncoder = mWristMotor.getEncoder();
    SparkMaxConfig wristConfig = new SparkMaxConfig();
    wristConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(Constants.Algae.kMaxWristCurrent)
        .inverted(true);

    mWristMotor.configure(
        wristConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Wrist PID
    mWristPIDController = new ProfiledPIDController(
        Constants.Algae.kWristP,
        Constants.Algae.kWristI,
        Constants.Algae.kWristD,
        new TrapezoidProfile.Constraints(
            Constants.Algae.kWristMaxVelocity,
            Constants.Algae.kWristMaxAcceleration));

    // Wrist Feedforward
    mWristFeedForward = new ArmFeedforward(
        Constants.Algae.kWristKS,
        Constants.Algae.kWristKG,
        Constants.Algae.kWristKV,
        Constants.Algae.kWristKA);

    // INTAKE
    mIntakeMotor = new SparkMax(Constants.Algae.algaeIntakeId, MotorType.kBrushless);
    SparkMaxConfig intakeConfig = new SparkMaxConfig();

    intakeConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.Algae.kMaxIntakeCurrent)
        .inverted(true);

    mIntakeMotor.configure(
        intakeConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  private static class PeriodicIO {
    double wrist_target_angle = 0.0;
    double wrist_voltage = 0.0;

    double intake_power = 0.0;

    IntakeState state = IntakeState.STOW;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    super.periodic();
    double pidCalc = mWristPIDController.calculate(getWristAngle(), mPeriodicIO.wrist_target_angle);
    double ffCalc = mWristFeedForward.calculate(Math.toRadians(getWristReferenceToHorizontal()),
        Math.toRadians(mWristPIDController.getSetpoint().velocity));

    mPeriodicIO.wrist_voltage = pidCalc + ffCalc;
    mWristMotor.set(mPeriodicIO.wrist_voltage);
    mIntakeMotor.set(mPeriodicIO.intake_power);
  }

  public void stop() {
    mPeriodicIO.wrist_voltage = 0.0;
    mPeriodicIO.wrist_target_angle = Constants.Algae.kStowAngle;

    mWristMotor.set(0.0);
    mIntakeMotor.set(0.0);
  }

  @Override
  public void outputTelemetry() {
    super.partsNT.setDouble("Wrist/Position", getWristAngle());
    super.partsNT.setDouble("Wrist/Target", mPeriodicIO.wrist_target_angle);
    super.partsNT.setDouble("Wrist/Current", mWristMotor.getOutputCurrent());
    super.partsNT.setDouble("Wrist/Output", mWristMotor.getAppliedOutput());
    super.partsNT.setDouble("Wrist/Voltage", mPeriodicIO.wrist_voltage);
    //super.partsNT.setDouble("Wrist/Frequency", mWristAbsEncoder.getFrequency());

    super.partsNT.setDouble("Intake/Current", mIntakeMotor.getOutputCurrent());
    super.partsNT.setDouble("Intake/Output", mIntakeMotor.getAppliedOutput());
    super.partsNT.setDouble("Intake/Power", mPeriodicIO.intake_power);
  }

  public void reset() {
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public void stow() {
    mPeriodicIO.wrist_target_angle = Constants.Algae.kStowAngle;

    mPeriodicIO.state = IntakeState.STOW;
    // mPeriodicIO.intake_power = 0.0;
  }

  public void grabAlgae() {
    mPeriodicIO.wrist_target_angle = Constants.Algae.kDeAlgaeAngle;
    mPeriodicIO.intake_power = Constants.Algae.kIntakeSpeed;

    mPeriodicIO.state = IntakeState.DEALGAE;
  }

  public void score() {
    if (mPeriodicIO.state == IntakeState.GROUND) {
      mPeriodicIO.intake_power = -Constants.Algae.kEjectSpeed;
    } else {
      mPeriodicIO.intake_power = Constants.Algae.kEjectSpeed;
    }
  }

  public void groundIntake() {
    mPeriodicIO.wrist_target_angle = Constants.Algae.kGroundIntakeAngle;
    mPeriodicIO.intake_power = Constants.Algae.kGroundIntakeSpeed;

    mPeriodicIO.state = IntakeState.GROUND;
  }

  public void stopAlgae() {
    mPeriodicIO.intake_power = 0.0;
    mPeriodicIO.wrist_target_angle = Constants.Algae.kStowAngle;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/

  public double getWristAngle() {
    return new PARTsUnit(mWristAbsEncoder.getPosition(), PARTsUnitType.Rotations).to(PARTsUnitType.Angle);
  }

  public double getWristReferenceToHorizontal() {
    return getWristAngle() - Constants.Algae.kWristOffset;
  }

  public IntakeState getState() {
    return mPeriodicIO.state;
  }

  public void setWristSpeed( double speed) {
    mWristMotor.set(speed);
  }

  public void setIntakeSpeed (double speed) {
    mIntakeMotor.set(speed);
  }

}

