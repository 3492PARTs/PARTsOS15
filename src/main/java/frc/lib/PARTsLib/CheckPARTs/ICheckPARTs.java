// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.PARTsLib.CheckPARTs;

/** Interface for testing a class, particularly subsystems.  */
public interface ICheckPARTs {
    public void report(PARTsError error);
}
