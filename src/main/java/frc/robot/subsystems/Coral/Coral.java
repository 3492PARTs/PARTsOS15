package frc.robot.subsystems.Coral;

// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.constants.CoralConstants;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Candle.CandleState;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.util.PARTs.PARTsCommandUtils;
import frc.robot.util.PARTs.Abstracts.PARTsSubsystem;

public abstract class Coral extends PARTsSubsystem {
  private Elevator elevator;

  /*-------------------------------- Private instance variables ---------------------------------*/
  protected PeriodicIO mPeriodicIO;
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

  public Coral(Candle candle, Elevator elevator) {
    super("Coral");

    this.candle = candle;
    this.elevator = elevator;

    mPeriodicIO = new PeriodicIO();

    new Trigger(this::isCoralInEntry).onTrue(Commands.runOnce(() -> candle.addState(CandleState.CORAL_ENTERING)))
        .onFalse(Commands.runOnce(() -> candle.removeState(CandleState.CORAL_ENTERING)));

    new Trigger(this::isCoralInExit).onTrue(Commands.runOnce(() -> candle.addState(CandleState.HAS_CORAL)))
        .onFalse(Commands.runOnce(() -> candle.removeState(CandleState.HAS_CORAL)));
  }

  protected static class PeriodicIO {
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

    checkErrors();

    if (mPeriodicIO.state != IntakeState.ERROR) {
      elevator.gantryBlocked(isCoralInEntry());
      checkAutoTasks();
      // Help us index a little more if its still detected in entry
      if (isCoralInEntry() && mPeriodicIO.state != IntakeState.INTAKE && mPeriodicIO.state != IntakeState.REVERSE) {
        index();
      }
    } else
      elevator.gantryBlocked(false);
  }

  @Override
  public void stop() {
    mPeriodicIO.rpm = 0.0;
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.state = IntakeState.NONE;
  }

  @Override
  public void outputTelemetry() {
    super.partsNT.putDouble("RPM/target", mPeriodicIO.rpm);

    if (mPeriodicIO.entryLaserMeasurement != null) {
      super.partsNT.putDouble("LaserEntry/distance", mPeriodicIO.entryLaserMeasurement.distance_mm);
      super.partsNT.putDouble("LaserEntry/ambient", mPeriodicIO.entryLaserMeasurement.ambient);
      super.partsNT.putDouble("LaserEntry/budget_ms", mPeriodicIO.entryLaserMeasurement.budget_ms);
      super.partsNT.putDouble("LaserEntry/status", mPeriodicIO.entryLaserMeasurement.status);
    }

    if (mPeriodicIO.entryLaserMeasurement != null)
      partsLogger.logDouble("LaserEntry/status", mPeriodicIO.entryLaserMeasurement.status);
    else
      partsLogger.logDouble("LaserEntry/status", -1);

    partsLogger.logBoolean("LaserEntry/laserMeasurementExists", (mPeriodicIO.entryLaserMeasurement != null));

    if (mPeriodicIO.exitLaserMeasurement != null) {
      super.partsNT.putDouble("LaserExit/distance", mPeriodicIO.exitLaserMeasurement.distance_mm);
      super.partsNT.putDouble("LaserExit/ambient", mPeriodicIO.exitLaserMeasurement.ambient);
      super.partsNT.putDouble("LaserExit/budget_ms", mPeriodicIO.exitLaserMeasurement.budget_ms);
      super.partsNT.putDouble("LaserExit/status", mPeriodicIO.exitLaserMeasurement.status);
    }

    if (mPeriodicIO.exitLaserMeasurement != null)
      partsLogger.logDouble("LaserExit/status", mPeriodicIO.exitLaserMeasurement.status);
    else
      partsLogger.logDouble("LaserExit/status", -1);

    partsLogger.logBoolean("LaserExit/laserMeasurementExists", (mPeriodicIO.exitLaserMeasurement != null));

    super.partsNT.putBoolean("LaserEntry/hasCoral", isCoralInEntry());
    super.partsNT.putBoolean("LaserExit/hasCoral", isCoralInExit());

    super.partsNT.putString("State", mPeriodicIO.state.toString());
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

  public Command commandIntake() {
    return PARTsCommandUtils.setCommandName("commandIntake",
        this.runOnce(() -> {
          mPeriodicIO.speed_diff = 0.0;
          mPeriodicIO.rpm = CoralConstants.kIntakeSpeed;
          mPeriodicIO.state = IntakeState.INTAKE;
        }));
  }

  public Command commandL4Intake() {
    return PARTsCommandUtils.setCommandName("commandL4Intake",
        this.runOnce(() -> {
          mPeriodicIO.speed_diff = 0.0;
          mPeriodicIO.rpm = CoralConstants.kInchIntakeSpeed;
        }));
  }

  public Command commandL4OutTake() {
    return PARTsCommandUtils.setCommandName("commandL4OutTake",
        this.runOnce(() -> {
          mPeriodicIO.speed_diff = 0.0;
          mPeriodicIO.rpm = -CoralConstants.kInchIntakeSpeed;
        }));
  }

  public Command commandAutoIntake() {
    return PARTsCommandUtils.setCommandName("commandAutoIntake",
        this.runOnce(() -> {
          mPeriodicIO.speed_diff = 0.0;
          mPeriodicIO.rpm = CoralConstants.kIntakeSpeed;
          mPeriodicIO.state = IntakeState.INTAKE;
        })).andThen(new WaitUntilCommand(() -> mPeriodicIO.state == IntakeState.READY));
  }

  public Command commandReverse() {
    return PARTsCommandUtils.setCommandName("commandReverse",
        this.runOnce(() -> {
          mPeriodicIO.speed_diff = 0.0;
          mPeriodicIO.rpm = CoralConstants.kReverseSpeed;
          mPeriodicIO.state = IntakeState.REVERSE;
        }));
  }

  public void index() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = CoralConstants.kIndexSpeed;
    mPeriodicIO.state = IntakeState.INDEX;

  }

  public void scoreL1() {
    mPeriodicIO.speed_diff = CoralConstants.kSpeedDifference;
    mPeriodicIO.rpm = CoralConstants.kL1Speed;
    mPeriodicIO.state = IntakeState.SCORE;
  }

  public void scoreL23() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = CoralConstants.kL23Speed;
    mPeriodicIO.state = IntakeState.SCORE;
  }

  public void scoreL4() {
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.rpm = CoralConstants.kL4Speed;
    mPeriodicIO.state = IntakeState.SCORE;
  }

  public void stopCoral() {
    mPeriodicIO.rpm = 0.0;
    mPeriodicIO.speed_diff = 0.0;
    mPeriodicIO.state = IntakeState.NONE;
  }

  public Command commandStop() {
    return PARTsCommandUtils.setCommandName("commandStop", super.runOnce(() -> stopCoral()));
  }

  public Command commandScore() {
    return PARTsCommandUtils.setCommandName("commandScore", this.runOnce(() -> {
      candle.addState(CandleState.SCORING);
      switch (elevator.getState()) {
        case STOW:
          scoreL1();
          break;
        case L4:
          scoreL4();
          break;
        default:
          scoreL23();
          break;
      }
    }));
  }

  public Command autoScore() {
    return this.runOnce(() -> {
      candle.addState(CandleState.SCORING);
      switch (elevator.getState()) {
        case STOW:
          scoreL1();
          break;
        case L4:
          scoreL4();
          break;
        default:
          scoreL23();
          break;
      }
    }).until(() -> !isCoralInExit());
  }

  public Command scoreCommand() {
    return PARTsCommandUtils.setCommandName("coralScoreCmd",
        commandScore().andThen(new WaitUntilCommand(() -> mPeriodicIO.state == IntakeState.NONE))
            .andThen(elevator.commandStow()));
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
        candle.addState(
            (mPeriodicIO.exitLaserMeasurement == null || !okStates.contains(mPeriodicIO.exitLaserMeasurement.status))
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
}
