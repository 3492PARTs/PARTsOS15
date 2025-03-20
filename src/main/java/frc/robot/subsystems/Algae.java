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

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.util.PARTsButtonBoxController;
import frc.robot.util.PARTsCommandController;
import frc.robot.util.PARTsNT;
import frc.robot.util.PARTsSubsystem;
import frc.robot.util.PARTsUnit;
import frc.robot.util.PARTsUnit.PARTsUnitType;

public class Algae extends PARTsSubsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private PeriodicIO mPeriodicIO;

  public enum IntakeState {
    NONE,
    STOW,
    REEFALGAE,
    GROUND
  }

  protected SparkMax mWristMotor;
  private final ProfiledPIDController mWristPIDController;
  //private final ArmFeedforward mWristFeedForward;

  private SparkMax mIntakeMotor;

  protected final RelativeEncoder mWristRelEncoder;

  public Algae() {
    super("Algae");

    mPeriodicIO = new PeriodicIO();

    // WRIST
    mWristMotor = new SparkMax(Constants.Algae.algaeWristId, MotorType.kBrushless);
    mWristRelEncoder = mWristMotor.getEncoder();
    SparkMaxConfig wristConfig = new SparkMaxConfig();
    wristConfig
        .idleMode(IdleMode.kBrake)
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
    mWristPIDController.setTolerance(Constants.Algae.kTolerance);

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

    reset();

    super.partsNT.putSmartDashboardSendable("Wrist PID", mWristPIDController);
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
    //test
    mWristPIDController.setGoal(mPeriodicIO.wrist_target_angle);
    double pidCalc = mWristPIDController.atGoal() ? 0
        : mWristPIDController.calculate(Math.toRadians(getWristAngle().getValue()),
            Math.toRadians(mPeriodicIO.wrist_target_angle));

    mPeriodicIO.wrist_voltage = -pidCalc; //ffCalc;

    setWristVoltage(mPeriodicIO.wrist_voltage);
    setIntakeSpeed(mPeriodicIO.intake_power);
  }

  @Override
  public void stop() {
    mPeriodicIO.wrist_voltage = 0.0;
    mPeriodicIO.wrist_target_angle = Constants.Algae.kStowAngle;

    setWristSpeed(0);
    setIntakeSpeed(0);
  }

  @Override
  public void outputTelemetry() {
    super.partsNT.setDouble("Wrist/Angle", getWristAngle().getValue());
    super.partsNT.setDouble("Wrist/Target", mPeriodicIO.wrist_target_angle);
    super.partsNT.setBoolean("Wrist/At Goal", mWristPIDController.atGoal());
    super.partsNT.setDouble("Wrist/Current", mWristMotor.getOutputCurrent());
    super.partsNT.setDouble("Wrist/Output", mWristMotor.getAppliedOutput());
    super.partsNT.setDouble("Wrist/Voltage", mPeriodicIO.wrist_voltage);
    super.partsNT.setDouble("Intake/Current", mIntakeMotor.getOutputCurrent());
    super.partsNT.setDouble("Intake/Output", mIntakeMotor.getAppliedOutput());
    super.partsNT.setDouble("Intake/Power", mPeriodicIO.intake_power);
    super.partsNT.setString("State", mPeriodicIO.state.toString());
  }

  @Override
  public void reset() {
    mWristRelEncoder.setPosition(0);
  }

  @Override
  public void log() {
    // TODO Auto-generated method stub
    //throw new UnsupportedOperationException("Unimplemented method 'log'");
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public Command stow() {
    return this.commandFactory("stow", this.runOnce(() -> {
      mPeriodicIO.wrist_target_angle = Constants.Algae.kStowAngle;

      mPeriodicIO.state = IntakeState.STOW;
      mPeriodicIO.intake_power = 0.0;
    }));

  }

  public Command grabReefAlgae() {
    return super.commandFactory("grabReefAlgae", this.runOnce(() -> {
      mPeriodicIO.wrist_target_angle = Constants.Algae.kDeAlgaeAngle;
      mPeriodicIO.intake_power = Constants.Algae.kReefIntakeSpeed;

      mPeriodicIO.state = IntakeState.REEFALGAE;
    }));
  }

  public Command stopAlgae() {
    return this.commandFactory("stopAlgae", this.runOnce(() -> {
      mPeriodicIO.intake_power = 0.0;
      mPeriodicIO.wrist_target_angle = Constants.Algae.kStowAngle;
    }));
  }

  public PARTsUnit getWristAngle() {

    return new PARTsUnit(
        new PARTsUnit(-1 * mWristRelEncoder.getPosition(), PARTsUnitType.Rotations).to(PARTsUnitType.Angle)
            / Constants.Algae.wristGearRatio,
        PARTsUnitType.Angle);
  }

  public double getWristReferenceToHorizontal() {
    return getWristAngle().getValue() - Constants.Algae.kWristOffset;
  }

  public IntakeState getState() {
    return mPeriodicIO.state;
  }

  public void setWristSpeed(double speed) {
    mWristMotor.set(speed);
  }

  public void setWristVoltage(double v) {
    mWristMotor.setVoltage(v);
  }

  public void setIntakeSpeed(double speed) {
    mIntakeMotor.set(speed);
  }

  public double getRPS() {
    return mWristRelEncoder.getVelocity() * 60 / Constants.Algae.wristGearRatio; // 16 is the gear reduction
  }
/* 
  public Command joystickAlgaeControl(PARTsButtonBoxController controller) {
    return super.commandFactory("joystickAlgaeControl", this.run(() -> {
      double speed = 0;
      if (controller.povTrigger0().getAsBoolean()) {
        speed = -0.5;
        mPeriodicIO.intake_power = Constants.Algae.kReefIntakeSpeed;
      } else if (controller.povTrigger180().getAsBoolean()) {
        speed = 0.5;
        mPeriodicIO.intake_power = 0;
      }

      setWristSpeed(speed);
    }).until(() -> !controller.povTrigger0().getAsBoolean() && !controller.povTrigger180().getAsBoolean())
        .andThen(() -> setWristSpeed(0)));
  }
        */

  public Command joystickAlgaeControl(PARTsCommandController controller) {
    return super.commandFactory("joystickAlgaeControl", this.run(() -> {
      double speed = 0;
      if (controller.povUp().getAsBoolean()) {
        speed = -0.5;
        mPeriodicIO.intake_power = Constants.Algae.kReefIntakeSpeed;

        if (getWristAngle().getValue() >= 132 ) {
          speed = 0;
        }
      } 
      else if (controller.povDown().getAsBoolean()) {
        speed = .5;
        mPeriodicIO.intake_power = 0;

          if (getWristAngle().getValue() <= 0) {
            speed = 0;
          }
      }

      setWristSpeed(speed);
    }).until(() -> !controller.povUp().getAsBoolean() && !controller.povDown().getAsBoolean())
        .andThen(() -> setWristSpeed(0)));
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/

}
