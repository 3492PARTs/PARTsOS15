// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import au.grapplerobotics.CanBridge;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.PARTsDashboard;
import frc.robot.util.PARTsLogger;
import frc.robot.util.PARTsNT;
import frc.robot.util.PARTsDashboard.DashboardTab;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  PARTsNT partsNT;
  PARTsLogger partsLogger;

  private final RobotContainer m_robotContainer;

  public Robot() {
    Logger.recordMetadata("ProjectName", "PARTsOS15");
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);

    if (isReal()) {
      // Logs to disk.
      //Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      setUseTiming(false);
      // Get replay log.
      String logPath = LogFileUtil.findReplayLog();
      // Read gotten log.
      Logger.setReplaySource(new WPILOGReader(logPath));
      // Sim log.
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    // This is needed for lasercan, without it causes robot to lag on boot
    CanBridge.runTCP();

    // Make elastic dashboard file available
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

    partsNT = new PARTsNT(this);
    partsLogger = new PARTsLogger();
    m_robotContainer = new RobotContainer();
    m_robotContainer.constructDashboard();

    partsLogger.logCommandScheduler();

    CameraServer.startAutomaticCapture();

    m_robotContainer.resetStartPose();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    partsNT.setDouble("Match Time", DriverStation.getMatchTime());
    m_robotContainer.outputTelemetry();
    m_robotContainer.log();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.stop();
    m_robotContainer.setCandleDisabledState();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    if (!Constants.Debug.debug) {
      PARTsDashboard.setTab(DashboardTab.AUTONOMOUS);
    }
    m_robotContainer.setIdleCandleState();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (!Constants.Debug.debug) {
      PARTsDashboard.setTab(DashboardTab.TELEOPERATED);
    }

    m_robotContainer.setIdleCandleState();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
