// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.cmds.coral;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.PARTsDrivetrain;
import frc.robot.subsystems.Candle.CandleState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L4ScoreCoral extends SequentialCommandGroup {

        /** Creates a new ScoreCoral. */
        public L4ScoreCoral(PARTsDrivetrain drivetrain, Elevator elevator,
                        Coral coral, Candle candle) {

                addCommands(
                                candle.addStateCommand(CandleState.SCORING),
                                elevator.elevatorToLevelCommand(ElevatorState.L4),
                                coral.score(),
                                new WaitCommand(.1),
                                elevator.elevatorToLevelCommand(ElevatorState.STOW));
        }
}
