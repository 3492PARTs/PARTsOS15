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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorState;

public class Elevator extends SubsystemBase {

  private static SparkMax leftElevatorMotor = new SparkMax(Constants.Elevator.leftElevatorId, MotorType.kBrushless);
  private static RelativeEncoder leftElevatorEncoder;

  private static SparkMax rightElevatorMotor = new SparkMax(Constants.Elevator.rightElevatorId, MotorType.kBrushless);
  private static RelativeEncoder rightElevatorEncoder;

  private PeriodicIO mPeriodicIO;

  /** Creates a new Elevator. */
  public Elevator() {

    mPeriodicIO = new PeriodicIO();

    SparkMaxConfig leftElevatorConfig = new SparkMaxConfig();
    leftElevatorConfig.idleMode(IdleMode.kBrake);
    leftElevatorMotor.configure(leftElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    SparkMaxConfig rightElevatorConfig = new SparkMaxConfig();
    rightElevatorConfig.idleMode(IdleMode.kBrake);
    rightElevatorConfig.follow(leftElevatorMotor);
    rightElevatorMotor.configure(rightElevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftElevatorEncoder = leftElevatorMotor.getEncoder();
    rightElevatorEncoder = rightElevatorMotor.getEncoder();

  }



  public void setSpeed(double speed) {
    rightElevatorMotor.set(speed);
    leftElevatorMotor.set(speed);
  }
  public double getEncoderDistance() {
    return leftElevatorEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void stop() {
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = 0.0;

  }

  private static class PeriodicIO {
    double elevator_target = 0.0;
    double elevator_power = 0.0;

    boolean is_elevator_pos_control = false;

    ElevatorState state = ElevatorState.STOW;
  }

  public ElevatorState getState() {
    return mPeriodicIO.state;
  }

  public void setElevatorPower(double power) {
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = power;
  }


  public void goToElevatorStow() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.StowHeight;
    mPeriodicIO.state = ElevatorState.STOW;
  }
  public void goToElevatorL2() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.L2Height;
    mPeriodicIO.state = ElevatorState.L2;
  }

  public void goToElevatorL3() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.L3Height;
    mPeriodicIO.state = ElevatorState.L3;
  }

  public void goToElevatorL4() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.L4Height;
    mPeriodicIO.state = ElevatorState.L4;
  }

  public void goToAlgaeLow() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.LowAlgaeHeight;
    mPeriodicIO.state = ElevatorState.A1;
  }

  public void goToAlgaeHigh() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.HighAlgaeHeight;
    mPeriodicIO.state = ElevatorState.A2;
  }

}
