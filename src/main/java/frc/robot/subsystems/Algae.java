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

public class Algae extends SubsystemBase {

  private static SparkMax algaeIntake = new SparkMax(Constants.Algae.algaeIntakeId, MotorType.kBrushless);
  private static RelativeEncoder algaeIntakEncoder;

  private static SparkMax algaeWrist = new SparkMax(Constants.Algae.algaeWristId, MotorType.kBrushless);
  private static RelativeEncoder algaeWristEncoder;

  public Algae() {
    SparkMaxConfig algaeIntakeConfig = new SparkMaxConfig();
    algaeIntakeConfig.idleMode(IdleMode.kBrake);
    algaeIntake.configure(algaeIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig algaeWristConfig = new SparkMaxConfig();
    algaeWristConfig.idleMode(IdleMode.kBrake);
    algaeWrist.configure(algaeWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    algaeIntakEncoder = algaeIntake.getEncoder();
    algaeWristEncoder = algaeWrist.getEncoder();
  }

  public void setSpeed(double speed) {
    algaeWrist.set(speed);
    algaeIntake.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
