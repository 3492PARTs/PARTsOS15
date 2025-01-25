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

public class Elevator extends SubsystemBase {

  private static SparkMax leftElevatorMotor = new SparkMax(Constants.Elevator.leftElevatorId, MotorType.kBrushless);
  private static RelativeEncoder leftElevatorEncoder;

  private static SparkMax rightElevatorMotor = new SparkMax(Constants.Elevator.rightElevatorId, MotorType.kBrushless);
  private static RelativeEncoder rightElevatorEncoder;

  /** Creates a new Elevator. */
  public Elevator() {

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
