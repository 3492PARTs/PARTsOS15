// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class PARTsSubsystem extends SubsystemBase implements IPARTsSubsystem {
  protected PARTsNT partsNT;
  protected PARTsLogger partsLogger;

  /**
   * Creates a new PARTsSubsystem.
   * <p>Comes with a PARTsNT generic instance.
   */
  public PARTsSubsystem() {
    partsNT = new PARTsNT();
    partsLogger = new PARTsLogger();
  }

  /**
   * Creates a new PARTsSubsystem.
   * <p>Comes with a PARTsNT instance based on the given class.
   */
  public PARTsSubsystem(Object o) {
    partsNT = new PARTsNT(o);
    partsLogger = new PARTsLogger(o);
  }

  /**
   * Creates a new PARTsSubsystem.
   * <p>Comes with a PARTsNT instance based on the given class name.
   */
  public PARTsSubsystem(String className) {
    partsNT = new PARTsNT(className);
    partsLogger = new PARTsLogger(className);
  }

}
