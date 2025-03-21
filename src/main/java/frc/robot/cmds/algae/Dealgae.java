// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.cmds.algae;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.PARTsDrivetrain;
import frc.robot.util.PARTsButtonBoxController;
import frc.robot.Constants;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Dealgae extends SequentialCommandGroup {
  private Pose2d deAlgaePose2d = Constants.Algae.deAlgaeDistance;

  
  /** Creates a new Dealgae. */
  public Dealgae( Elevator elevator, ElevatorState state, PARTsDrivetrain drivetrain,
  Algae algae, Candle candle, PARTsButtonBoxController buttonBoxController, Pose2d holdDistance) {
    addRequirements( elevator, algae);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(/*new ConditionalCommand(elevator.elevatorToLevelCommand(state), new PARTsAlignScoreAlgae(
      holdDistance,
        state, drivetrain, elevator, algae, candle,
        buttonBoxController
       ))*/);
  }
}
