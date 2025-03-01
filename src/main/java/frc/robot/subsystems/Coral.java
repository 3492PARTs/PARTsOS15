// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.Candle.CandleState;
import frc.robot.util.PARTsSubsystem;

public class Coral extends PARTsSubsystem {
  private Elevator elevator;

  /*-------------------------------- Private instance variables ---------------------------------*/
  private PeriodicIO mPeriodicIO;
  private final Candle candle;

  public enum IntakeState {
    NONE,
    INTAKE,
    REVERSE,
    INDEX,
    READY,
    SCORE,
    ERROR
  }

  private SparkMax mLeftMotor;
  private SparkMax mRightMotor;

  private LaserCan laserCAN;
  private Canandcolor canandcolor;

  public Coral(Candle candle, Elevator elevator) {
    super("Coral");

    this.candle = candle;
    this.elevator = elevator;

    mPeriodicIO = new PeriodicIO();

    mLeftMotor = new SparkMax(Constants.Coral.coralLeftMotorId, MotorType.kBrushless);
    mRightMotor = new SparkMax(Constants.Coral.coralRightMotorId, MotorType.kBrushless);

    SparkMaxConfig coralConfig = new SparkMaxConfig();

    coralConfig.idleMode(IdleMode.kBrake);

    mLeftMotor.configure(
        coralConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    mRightMotor.configure(
        coralConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    canandcolor = new Canandcolor(Constants.Coral.canAndColorId);
    canandcolor.setLampLEDBrightness(0);

    laserCAN = new LaserCan(Constants.Coral.laserCanId);
    try {
      laserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
      laserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(4, 4, 4, 4));
      laserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }

    new Trigger(this::isCoralInEntry).onTrue(Commands.runOnce(() -> candle.addState(CandleState.CORAL_ENTERING))).onFalse(Commands.runOnce(() -> candle.removeState(CandleState.CORAL_ENTERING)));

    new Trigger(this::isCoralInExit).onTrue(Commands.runOnce(() -> candle.addState(CandleState.HAS_CORAL))).onFalse(Commands.runOnce(() -> candle.removeState(CandleState.HAS_CORAL)));
  }

  private static class PeriodicIO {
    double rpm = 0.0;
    double speed_diff = 0.0;

    int index_debounce = 0;

    LaserCan.Measurement laserMeasurement = null;
    double colorMeasurement;

    IntakeState state = IntakeState.NONE;
    boolean error = false;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    mPeriodicIO.laserMeasurement = laserCAN.getMeasurement();
    mPeriodicIO.colorMeasurement = canandcolor.getProximity();

    // Trigger sub system error if exists
    // if there was an error but there isn't now remove error
    if (mPeriodicIO.laserMeasurement == null || mPeriodicIO.laserMeasurement.status != 0
        || !canandcolor.isConnected()) {
      if (!mPeriodicIO.error && mPeriodicIO.state != IntakeState.ERROR) {
        mPeriodicIO.error = true;
        mPeriodicIO.state = IntakeState.ERROR;
        candle.addState(CandleState.CORAL_ERROR);
      }

    } else {
      if (mPeriodicIO.error) {
        mPeriodicIO.error = false;
        mPeriodicIO.state = IntakeState.NONE;
        candle.removeState(CandleState.CORAL_ERROR);
      }
    }

    if (mPeriodicIO.state != IntakeState.ERROR) {
      elevator.setGantryBlock(isCoralInEntry());
      checkAutoTasks();
    } else
      elevator.setGantryBlock(false);

    mLeftMotor.set(mPeriodicIO.rpm - mPeriodicIO.speed_diff);
    mRightMotor.set(-mPeriodicIO.rpm);
  }

  @Override
  public void stop() {
    mPeriodicIO.rpm = 0.0;
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.state = IntakeState.NONE;
  }

  @Override
  public void outputTelemetry() {
    super.partsNT.setDouble("RPM/target", mPeriodicIO.rpm);

    if (mPeriodicIO.laserMeasurement != null) {
      super.partsNT.setDouble("Laser/distance", mPeriodicIO.laserMeasurement.distance_mm);
      super.partsNT.setDouble("Laser/ambient", mPeriodicIO.laserMeasurement.ambient);
      super.partsNT.setDouble("Laser/budget_ms", mPeriodicIO.laserMeasurement.budget_ms);
      super.partsNT.setDouble("Laser/status", mPeriodicIO.laserMeasurement.status);
    }

    super.partsNT.setBoolean("Laser/hasCoral", isCoralInEntry());

    super.partsNT.setDouble("Current/Left", mLeftMotor.getOutputCurrent());
    super.partsNT.setDouble("Current/Right", mRightMotor.getOutputCurrent());

    super.partsNT.setDouble("Output/Left", mLeftMotor.getAppliedOutput());
    super.partsNT.setDouble("Output/Right", mRightMotor.getAppliedOutput());

    super.partsNT.setBoolean("Canandcolor/hasCoral", isCoralInExit());
    super.partsNT.setDouble("Canandcolor/distance", mPeriodicIO.colorMeasurement);
    super.partsNT.setBoolean("Canandcolor/Connection", canandcolor.isConnected());

    super.partsNT.setString("State", mPeriodicIO.state.toString());
  }

  @Override
  public void reset() {
    stopCoral().schedule();
  }

  @Override
  public void log() {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method 'log'");
  }
  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public boolean isCoralInExit() {
    return mPeriodicIO.colorMeasurement <= 0.013;
  }

  public boolean isCoralInEntry() {
    if (mPeriodicIO.laserMeasurement != null)
      return mPeriodicIO.laserMeasurement.distance_mm <= 22;
    else
      return false;
  }

  public void setSpeed(double rpm) {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = rpm;
  }

  public Command intake() {
    return this.runOnce(() -> {
      mPeriodicIO.speed_diff = 0.0;
      mPeriodicIO.rpm = Constants.Coral.kIntakeSpeed;
      mPeriodicIO.state = IntakeState.INTAKE;
    });
  }

  public Command reverse() {
    return this.runOnce(() -> {
      mPeriodicIO.speed_diff = 0.0;
      mPeriodicIO.rpm = Constants.Coral.kReverseSpeed;
      mPeriodicIO.state = IntakeState.REVERSE;
    });
  }

  public Command index() {
    return this.runOnce(() -> {
      mPeriodicIO.speed_diff = 0.0;
      mPeriodicIO.rpm = Constants.Coral.kIndexSpeed;
      mPeriodicIO.state = IntakeState.INDEX;
    });
  }

  public Command scoreL1() {
    return this.runOnce(() -> {
      mPeriodicIO.speed_diff = Constants.Coral.kSpeedDifference;
      mPeriodicIO.rpm = Constants.Coral.kL1Speed;
      mPeriodicIO.state = IntakeState.SCORE;
    });
  }

  public Command scoreL24() {
    return this.runOnce(() -> {
      mPeriodicIO.speed_diff = 0.0;
      mPeriodicIO.rpm = Constants.Coral.kL24Speed;
      mPeriodicIO.state = IntakeState.SCORE;
    });
  }

  public Command stopCoral() {
    return this.runOnce(() -> {
      mPeriodicIO.rpm = 0.0;
      mPeriodicIO.speed_diff = 0.0;
      mPeriodicIO.state = IntakeState.NONE;
    });

  }

  public Command score() {
    return this.runOnce(() -> {
      candle.addState(CandleState.SCORING);
      switch (elevator.getState()) {
        case STOW:
          scoreL1().schedule();
          break;
        default:
          scoreL24().schedule();
          break;
      }
    });
  }

  public Command scoreCommand() {
    return score().andThen(new WaitUntilCommand(() -> mPeriodicIO.state == IntakeState.NONE))
        .andThen(elevator.goToElevatorStow());
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/

  private void checkAutoTasks() {
    switch (mPeriodicIO.state) {
      case INTAKE:
        if (isCoralInEntry()) {
          mPeriodicIO.index_debounce++;

          // Index for 10 loop run to get the coral in the right place
          if (mPeriodicIO.index_debounce > 10) {
            mPeriodicIO.index_debounce = 0;
            index().schedule();
          }
        }
        break;
      case INDEX:
        // pulls in slowly till we pass th sensor, stop after
        if (!isCoralInEntry()) {
          stopCoral().schedule();

          mPeriodicIO.state = IntakeState.READY;
        }
        break;
      case SCORE:
        // stop after the coral leaves the bot
        if (!isCoralInExit()) {
          mPeriodicIO.index_debounce++;

          // Let coral be gone for 10 loop runs then stow elevator
          if (mPeriodicIO.index_debounce > 10) {
            mPeriodicIO.index_debounce = 0;
            stopCoral().schedule();
            candle.removeState(CandleState.SCORING);
          }
        }
        break;
      default:
        break;
    }
  }

}
