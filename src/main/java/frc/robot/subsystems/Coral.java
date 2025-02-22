// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.CanandcolorSettings;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Candle.Color;

public class Coral extends PARTsSubsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private PeriodicIO mPeriodicIO;
  public final Candle mCandle;

  public enum IntakeState {
    NONE,
    INTAKE,
    REVERSE,
    INDEX,
    READY,
    SCORE
  }

  private SparkMax mLeftMotor;
  private SparkMax mRightMotor;

  private LaserCan laserCAN;
  private Canandcolor canandcolor;

  public Coral(Candle candle) {
    super("Coral");
    this.mCandle = candle;

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

    laserCAN = new LaserCan(Constants.Coral.laserCanId);
    try {
      laserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
      laserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      laserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  private static class PeriodicIO {
    double rpm = 0.0;
    double speed_diff = 0.0;

    int index_debounce = 0;

    LaserCan.Measurement laserMeasurement = null;
    double colorMeasurement;

    IntakeState state = IntakeState.NONE;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    mPeriodicIO.laserMeasurement = laserCAN.getMeasurement();
    mPeriodicIO.colorMeasurement = canandcolor.getProximity();

    checkAutoTasks();
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

    super.partsNT.setBoolean("Laser/hasCoral", isHoldingCoralViaLaserCAN());
    

    super.partsNT.setBoolean("Canandcolor/hasCoral", isHoldingCoralViaCAnandcolor());
    super.partsNT.setDouble("Canandcolor/distance", mPeriodicIO.colorMeasurement);
    super.partsNT.setBoolean("Canandcolor/Connection", canandcolor.isConnected());
  }

  @Override
  public void reset() {
    stopCoral();
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public boolean isHoldingCoralViaCAnandcolor() {
    return mPeriodicIO.colorMeasurement < 7.0;
  }

  public boolean isHoldingCoralViaLaserCAN() {
    if (mPeriodicIO.laserMeasurement != null)
      return mPeriodicIO.laserMeasurement.distance_mm < 7.0;
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

      mCandle.setColor(Color.BLUE);
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

      mCandle.setColor(Color.GREEN);
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

  /*---------------------------------- Custom Private Functions ---------------------------------*/

  private void checkAutoTasks() {
    switch (mPeriodicIO.state) {
      case INTAKE:
        if (isHoldingCoralViaLaserCAN()) {
          mPeriodicIO.index_debounce++;

          if (mPeriodicIO.index_debounce > 10) {
            mPeriodicIO.index_debounce = 0;
            index();
          }
        }
        break;
      case INDEX:
        if (!isHoldingCoralViaLaserCAN()) {
          stopCoral();

          mPeriodicIO.state = IntakeState.READY;
          mCandle.setColor(Color.PURPLE);
        }
        break;
      default:
        break;
    }
  }

  @Override
  public void log() {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method 'log'");
  }
}
