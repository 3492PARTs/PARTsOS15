// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.cmds.coral;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PARTsDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.util.PARTsButtonBoxController;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConditionalAlign extends SequentialCommandGroup {
  /** Creates a new ConditionalAlign. */
  public ConditionalAlign(BooleanSupplier supplier, Elevator elevator, ElevatorState state, PARTsDrivetrain drivetrain,
      Coral coral, Candle candle, PARTsButtonBoxController buttonBoxController, Pose2d holdDistance, Vision vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ConditionalCommand(elevator.elevatorToLevelCommand(state),
        new AlignScoreCoral(holdDistance, state, drivetrain, elevator, coral, candle, vision), supplier));
  }
}
