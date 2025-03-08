// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.PARTsSubsystem;

public class Climber extends PARTsSubsystem {
  /** Creates a new Climber. */
  private TalonFX climberMotor;

  public Climber() {
    //TODO; change to Default Name
    climberMotor = new TalonFX(Constants.Climber.climberId, "rio");
    climberMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setSpeed(double speed) {
    climberMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void outputTelemetry() {
    // TODO Auto-generated method stub
    //throw new UnsupportedOperationException("Unimplemented method 'outputTelemetry'");
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    //throw new UnsupportedOperationException("Unimplemented method 'stop'");
  }

  @Override
  public void reset() {
    // TODO Auto-generated method stub
    //throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }

  @Override
  public void log() {
    // TODO Auto-generated method stub
    //throw new UnsupportedOperationException("Unimplemented method 'log'");
  }
}
