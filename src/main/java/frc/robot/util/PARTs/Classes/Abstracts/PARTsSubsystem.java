package frc.robot.util.PARTs.Classes.Abstracts;

// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.PARTs.Classes.PARTsLogger;
import frc.robot.util.PARTs.Classes.PARTsNT;
import frc.robot.util.PARTs.Classes.PARTsPreferences;
import frc.robot.util.PARTs.Interfaces.IPARTsSubsystem;

public abstract class PARTsSubsystem extends SubsystemBase implements IPARTsSubsystem {
  protected PARTsNT partsNT;
  protected PARTsLogger partsLogger;
  protected PARTsPreferences partsPrefrences;

  /**
   * Creates a new PARTsSubsystem.
   * <p>Comes with a PARTsNT generic instance.
   */
  public PARTsSubsystem() {
    partsNT = new PARTsNT(this.getName());
    partsLogger = new PARTsLogger(this.getName());
    partsPrefrences = new PARTsPreferences();
  }

  /**
   * Creates a new PARTsSubsystem.
   * <p>Comes with a PARTsNT instance based on the given class.
   */
  public PARTsSubsystem(Object o) {
    partsNT = new PARTsNT(o);
    partsLogger = new PARTsLogger(o);
    partsPrefrences = new PARTsPreferences();
  }

  /**
   * Creates a new PARTsSubsystem.
   * <p>Comes with a PARTsNT instance based on the given class name.
   */
  public PARTsSubsystem(String className) {
    partsNT = new PARTsNT(className);
    partsLogger = new PARTsLogger(className);
    partsPrefrences = new PARTsPreferences();
  }
}
