// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PARTsSubsystem;

public class Buttonbox extends PARTsSubsystem {
  private Joystick joystick;
  /** Creates a new Buttonbox. */
  public Buttonbox(Joystick joystick) {
    super("Buttonbox");
    this.joystick = joystick;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void outputTelemetry() {
    super.partsNT.setInteger("button count", joystick.getButtonCount()); 
   // super.partsNT.setBoolean("Button zero", joystick.getRawButton(0)); keep in case for copying
    super.partsNT.setBoolean("HANDLE", joystick.getRawButton(1));
    super.partsNT.setBoolean("CRUISE", joystick.getRawButton(2));
    super.partsNT.setBoolean("FLASH", joystick.getRawButton(3));
    super.partsNT.setBoolean("AUDIO", joystick.getRawButton(4));
    super.partsNT.setBoolean("WIPERS", joystick.getRawButton(5));
    super.partsNT.setBoolean("MAP", joystick.getRawButton(6));
    super.partsNT.setBoolean("LIGHT ON ", joystick.getRawButton(7));
    super.partsNT.setBoolean("TALK ON  ", joystick.getRawButton(8));
    
    // TODO Auto-generated method stub


  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method 'stop'");
  }

  @Override
  public void reset() {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }

  @Override
  public void log() {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method 'log'");
  }
}
