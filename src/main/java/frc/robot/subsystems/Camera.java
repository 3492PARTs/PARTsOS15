// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSource;
import frc.robot.util.PARTsSubsystem;

public class Camera extends PARTsSubsystem {
  /** Creates a new Camera. */
  public Camera() {
    CameraServer.startAutomaticCapture();
  }
  public VideoSource getVideoSource() {
    return CameraServer.getVideo().getSource();
  }

  @Override
  public void periodic() {}

  @Override
  public void outputTelemetry() {}
  
  @Override
  public void stop() {
    CameraServer.getServer().close();
  }

  @Override
  public void reset() {}

  @Override
  public void log() {}
}
