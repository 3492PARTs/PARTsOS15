// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorJoystick extends Command {
  private Elevator m_Elevator;
  private CommandXboxController m_Controller;
  /** Creates a new ElevatorJoystick. */
  public ElevatorJoystick(Elevator elevator, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Elevator = elevator;
    m_Controller = controller;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(m_Controller.getRightY()) > 0.1) {
      if (!m_Elevator.getLimitSwitch()) {
        m_Elevator.setSpeed(m_Controller.getRightY());
      }
      else if (m_Controller.getRightY() > 0) {
        m_Elevator.setSpeed(m_Controller.getRightY());
      }
    }
    else {
      m_Elevator.setSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elevator.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
