// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.PARTsLogger;
import frc.robot.util.PARTsNT;
import au.grapplerobotics.CanBridge;



public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  PARTsNT partsNT;
  PARTsLogger partsLogger;

  private final RobotContainer m_robotContainer;

  public Robot() {
    CanBridge.runTCP();

    partsNT = new PARTsNT(this);
    partsLogger = new PARTsLogger();
    m_robotContainer = new RobotContainer();
    // Set the scheduler to log Shuffleboard events for command initialize, interrupt, finish
    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> {
              partsLogger.logString(command.getName(), "Command initialized");
              Shuffleboard.addEventMarker(
                  "Command initialized", command.getName(), EventImportance.kNormal);
              //System.out.println("Command initialized " + command.getName());
            });
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command -> {
              partsLogger.logString(command.getName(), "Command interrupted");
              Shuffleboard.addEventMarker(
                  "Command interrupted", command.getName(), EventImportance.kNormal);
              //System.out.println("Command interrupted " + command.getName());
            });
    CommandScheduler.getInstance()
        .onCommandFinish(
            command -> {
              partsLogger.logString(command.getName(), "Command finished");
              Shuffleboard.addEventMarker(
                  "Command finished", command.getName(), EventImportance.kNormal);
              //System.out.println("Command finished " + command.getName());
            });
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    partsNT.setDouble("Match Time", DriverStation.getMatchTime());
    m_robotContainer.outputTelemetry();
  }

  @Override
  public void disabledInit() {
    m_robotContainer.stop();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
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
