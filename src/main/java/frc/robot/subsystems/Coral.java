// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Coral extends SubsystemBase {

  private static SparkMax coralRightMotor = new SparkMax(Constants.Coral.coralRightMotorId, MotorType.kBrushless);
  private static RelativeEncoder coralRightMotorEncoder;

  private static SparkMax coralLeftMotor = new SparkMax(Constants.Coral.coralLeftMotorId, MotorType.kBrushless);
  private static RelativeEncoder coralLeftMotorEncoder;
  /** Creates a new Coral. */
  public Coral() {
     SparkMaxConfig coralRightMotorConfig = new SparkMaxConfig();
    coralRightMotorConfig.idleMode(IdleMode.kBrake);
    coralRightMotor.configure(coralRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig coralLeftMotorConfig = new SparkMaxConfig();
    coralLeftMotorConfig.idleMode(IdleMode.kBrake);
    coralLeftMotorConfig.follow(coralRightMotor);
    coralLeftMotor.configure(coralLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    coralRightMotorEncoder = coralRightMotor.getEncoder();
    coralLeftMotorEncoder = coralLeftMotor.getEncoder();
  }

  public void setSpeed(double speed) {
    coralRightMotor.set(speed);
    coralLeftMotor.set(speed);
  }

  public double getEncoderDistance() {
    return coralLeftMotorEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
