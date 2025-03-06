// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.testing;

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
    super.partsNT.setBoolean("ESC ", joystick.getRawButton(9));
    super.partsNT.setBoolean("ENTER  ", joystick.getRawButton(10));
    super.partsNT.setBoolean("ENGINESTART ", joystick.getRawButton(11));
    super.partsNT.setBoolean("NUKE  ", joystick.getRawButton(12));
    super.partsNT.setBoolean("POSITIVE-1", joystick.getRawButton(13));
    super.partsNT.setBoolean("NEGATIVE-1", joystick.getRawButton(14));
    super.partsNT.setBoolean("POSITIVE-2", joystick.getRawButton(15));
    super.partsNT.setBoolean("NEGATIVE-2", joystick.getRawButton(16));
    super.partsNT.setBoolean("POSITIVE-3", joystick.getRawButton(17));
    super.partsNT.setBoolean("NEGATIVE-3", joystick.getRawButton(18));
    super.partsNT.setBoolean("POSITIVE-4", joystick.getRawButton(19));
    super.partsNT.setBoolean("NEGATIVE-4", joystick.getRawButton(20));
    super.partsNT.setBoolean("ABS-RIGHT", joystick.getRawButton(21));
    super.partsNT.setBoolean("ABS-LEFT", joystick.getRawButton(22));
    super.partsNT.setBoolean("TC-RIGHT", joystick.getRawButton(23));
    super.partsNT.setBoolean("TC-LEFT", joystick.getRawButton(24));
    super.partsNT.setBoolean("ABS-CLICK", joystick.getRawButton(25));
    super.partsNT.setBoolean("TC-CLICK", joystick.getRawButton(26));
    super.partsNT.setInteger("Joystick", joystick.getPOV(0));

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
