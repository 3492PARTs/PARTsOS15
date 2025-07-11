package frc.robot.subsystems.Coral;

// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.util.ArrayList;
import java.util.Arrays;

import au.grapplerobotics.LaserCan;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.states.CandleState;
import frc.robot.states.CoralState;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.util.PARTs.Classes.PARTsCommandUtils;
import frc.robot.util.PARTs.Classes.Abstracts.PARTsSubsystem;

public abstract class Coral extends PARTsSubsystem {
  protected CoralState coralState = CoralState.NONE;
  protected LaserCan.Measurement entryLaserMeasurement;
  protected LaserCan.Measurement exitLaserMeasurement;

  /*-------------------------------- Private instance variables ---------------------------------*/
  private Elevator elevator;
  private final Candle candle;

  public Coral(Candle candle, Elevator elevator) {
    super("Coral");

    this.candle = candle;
    this.elevator = elevator;

    new Trigger(this::isCoralInEntry)
        .onTrue(Commands.runOnce(() -> candle.addState(CandleState.CORAL_ENTERING)).ignoringDisable(true))
        .onFalse(Commands.runOnce(() -> candle.removeState(CandleState.CORAL_ENTERING)).ignoringDisable(true));

    new Trigger(this::isCoralInExit)
        .onTrue(Commands.runOnce(() -> candle.addState(CandleState.HAS_CORAL)).ignoringDisable(true))
        .onFalse(Commands.runOnce(() -> candle.removeState(CandleState.HAS_CORAL)).ignoringDisable(true));
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {

    checkErrors();

    if (coralState != CoralState.ERROR) {
      elevator.gantryBlocked(isCoralInEntry());

      switch (coralState) {
        case INTAKE:
          if (isCoralInEntry()) {
            index();
          }
          break;
        case INDEX:
          // pulls in slowly till we pass the sensor, stop after
          if (!isCoralInEntry() && isCoralInExit()) {
            stopCoral();
            coralState = CoralState.READY;
          }
          break;
        case L1:
        case L23:
        case L4:
          // stop after the coral leaves the bot
          if (!isCoralInExit()) {
            stopCoral();
            candle.removeState(CandleState.SCORING);
          }
          break;
        default:
          break;
      }

      // Help us index a little more if its still detected in entry
      if (isCoralInEntry()
          && !new ArrayList<>(Arrays.asList(CoralState.INTAKE, CoralState.REVERSE)).contains(coralState)) {
        index();
      }
    } else
      elevator.gantryBlocked(false); // if there is an error, don't report state to gantry in elevator
  }

  @Override
  public void stop() {
    coralState = CoralState.NONE;
  }

  @Override
  public void outputTelemetry() {
    super.partsNT.putDouble("RPM/target", coralState.getSpeed());

    if (entryLaserMeasurement != null) {
      super.partsNT.putDouble("LaserEntry/distance", entryLaserMeasurement.distance_mm);
      super.partsNT.putDouble("LaserEntry/ambient", entryLaserMeasurement.ambient);
      super.partsNT.putDouble("LaserEntry/budget_ms", entryLaserMeasurement.budget_ms);
      super.partsNT.putDouble("LaserEntry/status", entryLaserMeasurement.status);
    }

    if (entryLaserMeasurement != null)
      partsLogger.logDouble("LaserEntry/status", entryLaserMeasurement.status);
    else
      partsLogger.logDouble("LaserEntry/status", -1);

    partsLogger.logBoolean("LaserEntry/laserMeasurementExists", (entryLaserMeasurement != null));

    if (exitLaserMeasurement != null) {
      super.partsNT.putDouble("LaserExit/distance", exitLaserMeasurement.distance_mm);
      super.partsNT.putDouble("LaserExit/ambient", exitLaserMeasurement.ambient);
      super.partsNT.putDouble("LaserExit/budget_ms", exitLaserMeasurement.budget_ms);
      super.partsNT.putDouble("LaserExit/status", exitLaserMeasurement.status);
    }

    if (exitLaserMeasurement != null)
      partsLogger.logDouble("LaserExit/status", exitLaserMeasurement.status);
    else
      partsLogger.logDouble("LaserExit/status", -1);

    partsLogger.logBoolean("LaserExit/laserMeasurementExists", (exitLaserMeasurement != null));

    super.partsNT.putBoolean("LaserEntry/hasCoral", isCoralInEntry());
    super.partsNT.putBoolean("LaserExit/hasCoral", isCoralInExit());

    super.partsNT.putString("State", coralState.toString());
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
    if (exitLaserMeasurement != null)
      return exitLaserMeasurement.distance_mm <= 22;
    else
      return false;
  }

  public boolean isCoralInEntry() {
    if (entryLaserMeasurement != null)
      return entryLaserMeasurement.distance_mm <= 22;
    else
      return false;
  }

  public Command commandIntake() {
    return PARTsCommandUtils.setCommandName("commandIntake",
        this.runOnce(() -> {
          intake();
        }));
  }

  public Command commandInchIntake() {
    return PARTsCommandUtils.setCommandName("commandInchIntake",
        this.runOnce(() -> {
          inchIntake();
        }));
  }

  public Command commandInchReverse() {
    return PARTsCommandUtils.setCommandName("commandInchReverse",
        this.runOnce(() -> {
          inchReverse();
        }));
  }

  public Command commandAutoIntake() {
    return PARTsCommandUtils.setCommandName("commandAutoIntake",
        this.runOnce(() -> {
          intake();
        })).andThen(new WaitUntilCommand(() -> coralState == CoralState.READY));
  }

  public Command commandReverse() {
    return PARTsCommandUtils.setCommandName("commandReverse",
        this.runOnce(() -> {
          reverseCoral();
        }));
  }

  public Command commandScoreL1() {
    return PARTsCommandUtils.setCommandName("commandReverse",
        this.runOnce(() -> {
          scoreL1();
        }));
  }

  public Command commandScoreL23() {
    return PARTsCommandUtils.setCommandName("commandReverse",
        this.runOnce(() -> {
          scoreL23();
        }));
  }

  public Command commandScoreL4() {
    return PARTsCommandUtils.setCommandName("commandReverse",
        this.runOnce(() -> {
          scoreL4();
        }));
  }

  public void index() {
    coralState = CoralState.INDEX;
  }

  public void intake() {
    coralState = CoralState.INTAKE;
  }

  public void scoreL1() {
    coralState = CoralState.L1;
  }

  public void scoreL23() {
    coralState = CoralState.L23;
  }

  public void scoreL4() {
    coralState = CoralState.L4;
  }

  public void inchIntake() {
    coralState = CoralState.INCH_INTAKE;
  }

  public void inchReverse() {
    coralState = CoralState.INCH_REVERSE;
  }

  public void stopCoral() {
    coralState = CoralState.NONE;
  }

  public void reverseCoral() {
    coralState = CoralState.REVERSE;
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

  public Command commandAutoScore() {
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
        commandScore().andThen(new WaitUntilCommand(() -> coralState == CoralState.NONE))
            .andThen(elevator.commandStow()));
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/

  private void checkErrors() {
    ArrayList<Integer> okStates = new ArrayList<Integer>();
    okStates.add(0);
    okStates.add(2);

    // Trigger sub system error if exists
    // if there was an error but there isn't now remove error
    if (entryLaserMeasurement == null || !okStates.contains(entryLaserMeasurement.status)
        || exitLaserMeasurement == null || !okStates.contains(exitLaserMeasurement.status)) {
      if (coralState != CoralState.ERROR) {
        coralState = CoralState.ERROR;
        candle.addState(
            (exitLaserMeasurement == null || !okStates.contains(exitLaserMeasurement.status))
                ? CandleState.CORAL_LASER_EXIT_ERROR
                : CandleState.CORAL_LASER_ENTRY_ERROR);
      }

    } else {
      if (coralState == CoralState.ERROR) {
        coralState = CoralState.NONE;
        candle.removeState(CandleState.CORAL_LASER_EXIT_ERROR);
        candle.removeState(CandleState.CORAL_LASER_ENTRY_ERROR);
      }
    }
  }
}
