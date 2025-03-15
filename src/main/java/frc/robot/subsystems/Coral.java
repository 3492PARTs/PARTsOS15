// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

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

  private LaserCan entrySensor;
  private LaserCan exitSensor;

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

    entrySensor = new LaserCan(Constants.Coral.laserCanId);
    try {
      entrySensor.setRangingMode(LaserCan.RangingMode.SHORT);
      entrySensor.setRegionOfInterest(new LaserCan.RegionOfInterest(4, 4, 4, 4));
      entrySensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }

    exitSensor = new LaserCan(Constants.Coral.laserCan2Id);
    try {
      exitSensor.setRangingMode(LaserCan.RangingMode.SHORT);
      exitSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(4, 4, 4, 4));
      exitSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }

    new Trigger(this::isCoralInEntry).onTrue(Commands.runOnce(() -> candle.addState(CandleState.CORAL_ENTERING)))
    .onFalse(Commands.runOnce(() -> candle.removeState(CandleState.CORAL_ENTERING)));

new Trigger(this::isCoralInExit).onTrue(Commands.runOnce(() -> candle.addState(CandleState.HAS_CORAL)))
    .onFalse(Commands.runOnce(() -> candle.removeState(CandleState.HAS_CORAL)));
  }

  private static class PeriodicIO {
    double rpm = 0.0;
    double speed_diff = 0.0;

    int index_debounce = 0;

    LaserCan.Measurement entryLaserMeasurement = null;
    LaserCan.Measurement exitLaserMeasurement = null;

    IntakeState state = IntakeState.NONE;
    boolean error = false;

    CandleState candleState = null;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    mPeriodicIO.entryLaserMeasurement = entrySensor.getMeasurement();
    mPeriodicIO.exitLaserMeasurement = exitSensor.getMeasurement();

    checkErrors();

    if (mPeriodicIO.state != IntakeState.ERROR) {
      elevator.setGantryBlock(isCoralInEntry());
      checkAutoTasks();
      // Help us index a little more if its still detected in entry
      if (isCoralInEntry() && mPeriodicIO.state != IntakeState.INTAKE) {
        mPeriodicIO.state = IntakeState.INDEX;
      }
    } else
      elevator.setGantryBlock(false);

    mLeftMotor.set(mPeriodicIO.rpm - mPeriodicIO.speed_diff);
    mRightMotor.set(-mPeriodicIO.rpm);

    //setCoralCandleState();
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

    if (mPeriodicIO.entryLaserMeasurement != null) {
      super.partsNT.setDouble("LaserEntry/distance", mPeriodicIO.entryLaserMeasurement.distance_mm);
      super.partsNT.setDouble("LaserEntry/ambient", mPeriodicIO.entryLaserMeasurement.ambient);
      super.partsNT.setDouble("LaserEntry/budget_ms", mPeriodicIO.entryLaserMeasurement.budget_ms);
      super.partsNT.setDouble("LaserEntry/status", mPeriodicIO.entryLaserMeasurement.status);
    }

    if (mPeriodicIO.entryLaserMeasurement != null)
      partsLogger.logDouble("LaserEntry/status", mPeriodicIO.entryLaserMeasurement.status);
    else
      partsLogger.logDouble("LaserEntry/status", -1);

    partsLogger.logBoolean("LaserEntry/laserMeasurementExists", (mPeriodicIO.entryLaserMeasurement != null));

    if (mPeriodicIO.exitLaserMeasurement != null) {
      super.partsNT.setDouble("LaserExit/distance", mPeriodicIO.exitLaserMeasurement.distance_mm);
      super.partsNT.setDouble("LaserExit/ambient", mPeriodicIO.exitLaserMeasurement.ambient);
      super.partsNT.setDouble("LaserExit/budget_ms", mPeriodicIO.exitLaserMeasurement.budget_ms);
      super.partsNT.setDouble("LaserExit/status", mPeriodicIO.exitLaserMeasurement.status);
    }

    if (mPeriodicIO.exitLaserMeasurement != null)
      partsLogger.logDouble("LaserExit/status", mPeriodicIO.exitLaserMeasurement.status);
    else
      partsLogger.logDouble("LaserExit/status", -1);

    partsLogger.logBoolean("LaserExit/laserMeasurementExists", (mPeriodicIO.exitLaserMeasurement != null));

    super.partsNT.setBoolean("LaserEntry/hasCoral", isCoralInEntry());
    super.partsNT.setBoolean("LaserExit/hasCoral", isCoralInExit());

    super.partsNT.setDouble("Current/Left", mLeftMotor.getOutputCurrent());
    super.partsNT.setDouble("Current/Right", mRightMotor.getOutputCurrent());

    super.partsNT.setDouble("Output/Left", mLeftMotor.getAppliedOutput());
    super.partsNT.setDouble("Output/Right", mRightMotor.getAppliedOutput());

    super.partsNT.setString("State", mPeriodicIO.state.toString());
  }

  @Override
  public void reset() {
    stopCoral();
  }

  @Override
  public void log() {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method 'log'");
  }
  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public boolean isCoralInExit() {
    if (mPeriodicIO.exitLaserMeasurement != null)
      return mPeriodicIO.exitLaserMeasurement.distance_mm <= 22;
    else
      return false;
  }

  public boolean isCoralInEntry() {
    if (mPeriodicIO.entryLaserMeasurement != null)
      return mPeriodicIO.entryLaserMeasurement.distance_mm <= 22;
    else
      return false;
  }

  public void setSpeed(double rpm) {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = rpm;
  }

  public Command intake() {
    return super.commandFactory("coralIntake",
        this.runOnce(() -> {
          mPeriodicIO.speed_diff = 0.0;
          mPeriodicIO.rpm = Constants.Coral.kIntakeSpeed;
          mPeriodicIO.state = IntakeState.INTAKE;
        }));
  }

  public Command reverse() {
    return super.commandFactory("coralReverse",
        this.runOnce(() -> {
          mPeriodicIO.speed_diff = 0.0;
          mPeriodicIO.rpm = Constants.Coral.kReverseSpeed;
          mPeriodicIO.state = IntakeState.REVERSE;
        }));
  }

  public void index() {
          mPeriodicIO.speed_diff = 0.0;
          mPeriodicIO.rpm = Constants.Coral.kIndexSpeed;
          mPeriodicIO.state = IntakeState.INDEX;

  }

  public void scoreL1() {
    // return this.runOnce(() -> {
    mPeriodicIO.speed_diff = Constants.Coral.kSpeedDifference;
    mPeriodicIO.rpm = Constants.Coral.kL1Speed;
    mPeriodicIO.state = IntakeState.SCORE;
  }

  public void scoreL24() {
    // return this.runOnce(() -> {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = Constants.Coral.kL24Speed;
    mPeriodicIO.state = IntakeState.SCORE;
  }

  public void scoreL4() {
    // return this.runOnce(() -> {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = -0.2;
    mPeriodicIO.state = IntakeState.SCORE;
  }

  public void stopCoral() {
    mPeriodicIO.rpm = 0.0;
          mPeriodicIO.speed_diff = 0.0;
          mPeriodicIO.state = IntakeState.NONE;

  }

  public Command stopCoralCommand() {
    return super.commandFactory("stopCoralCommand", super.runOnce(() -> stopCoral()));
  }

  public Command score() {
    return this.runOnce(() -> {
      candle.addState(CandleState.SCORING);
      switch (elevator.getState()) {
        case STOW:
          scoreL1();
          break;
        default:
          scoreL24();
          break;
      }
    });
  }

  public Command autoScore() {
    return this.runOnce(() -> {
      candle.addState(CandleState.SCORING);
      switch (elevator.getState()) {
        case STOW:
          scoreL1();
          break;
        default:
          scoreL24();
          break;
      }
    }).until(() -> !isCoralInEntry());
  }

  public Command scoreCommand() {
    return super.commandFactory("coralScoreCmd",
        score().andThen(new WaitUntilCommand(() -> mPeriodicIO.state == IntakeState.NONE))
            .andThen(elevator.goToElevatorStow()));
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/

  private void checkAutoTasks() {
    switch (mPeriodicIO.state) {
      case INTAKE:
        if (isCoralInEntry()) {
          mPeriodicIO.index_debounce++;

          // Index for 10 loop run to get the coral in the right place, or check if in the
          // exit incase it happened too fast
          if (mPeriodicIO.index_debounce > 10 || isCoralInExit()) {
            mPeriodicIO.index_debounce = 0;

          }
          index();
        } else {
          mPeriodicIO.index_debounce = 0;
        }
        break;
      case INDEX:
        // pulls in slowly till we pass the sensor, stop after
        if (!isCoralInEntry() && isCoralInExit()) {
          mPeriodicIO.index_debounce++;

          if (mPeriodicIO.index_debounce > 1) {
            mPeriodicIO.index_debounce = 0;

          }

          stopCoral();
          mPeriodicIO.state = IntakeState.READY;
        } else {
          mPeriodicIO.index_debounce = 0;
        }
        break;
      case SCORE:
        // stop after the coral leaves the bot
        if (!isCoralInExit()) {
          mPeriodicIO.index_debounce++;

          // Let coral be gone for 10 loop runs
          if (mPeriodicIO.index_debounce > 10) {
            mPeriodicIO.index_debounce = 0;

          }
          stopCoral();
          candle.removeState(CandleState.SCORING);
        }
        break;
      default:
        break;
    }
  }

  private void checkErrors() {
    ArrayList<Integer> okStates = new ArrayList<Integer>();
    okStates.add(0);
    okStates.add(2);

    // Trigger sub system error if exists
    // if there was an error but there isn't now remove error
    if (mPeriodicIO.entryLaserMeasurement == null || !okStates.contains(mPeriodicIO.entryLaserMeasurement.status)
        || mPeriodicIO.exitLaserMeasurement == null || !okStates.contains(mPeriodicIO.exitLaserMeasurement.status)) {
      if (!mPeriodicIO.error && mPeriodicIO.state != IntakeState.ERROR) {
        mPeriodicIO.error = true;
        mPeriodicIO.state = IntakeState.ERROR;
        candle.addState((mPeriodicIO.exitLaserMeasurement == null || !okStates.contains(mPeriodicIO.exitLaserMeasurement.status))
            ? CandleState.CORAL_LASER_EXIT_ERROR
            : CandleState.CORAL_LASER_ENTRY_ERROR);
      }

    } else {
      if (mPeriodicIO.error) {
        mPeriodicIO.error = false;
        mPeriodicIO.state = IntakeState.NONE;
        candle.removeState(CandleState.CORAL_LASER_EXIT_ERROR);
        candle.removeState(CandleState.CORAL_LASER_ENTRY_ERROR);
      }
    }
  }

  private void setCoralCandleState() {
    if (isCoralInEntry()) {
      if (mPeriodicIO.candleState != CandleState.CORAL_ENTERING) {
        mPeriodicIO.candleState = CandleState.CORAL_ENTERING;
        candle.addState(mPeriodicIO.candleState);
      }
    } else {
      if (mPeriodicIO.candleState == CandleState.CORAL_ENTERING) {
        candle.removeState(mPeriodicIO.candleState);
        mPeriodicIO.candleState = null;
      }
    }

    if (isCoralInExit()) {
      if (mPeriodicIO.candleState != CandleState.HAS_CORAL) {
        mPeriodicIO.candleState = CandleState.HAS_CORAL;
        candle.addState(mPeriodicIO.candleState);
      }
    } else {
      if (mPeriodicIO.candleState == CandleState.HAS_CORAL) {
        candle.removeState(mPeriodicIO.candleState);
        mPeriodicIO.candleState = null;
      }
    }
  }
}
