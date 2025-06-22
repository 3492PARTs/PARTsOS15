// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.cmds.coral;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.PARTsDrivetrain;
import frc.robot.util.PARTsButtonBoxController;
import frc.robot.util.PARTsUnit;
import frc.robot.util.PARTsUnit.PARTsUnitType;
import frc.robot.subsystems.Candle.CandleState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PARTsAlignScoreCoral extends SequentialCommandGroup {

        /** Creates a new ScoreCoral. */
        public PARTsAlignScoreCoral(Pose2d holdDistance, ElevatorState level, PARTsDrivetrain drivetrain,
                        Elevator elevator,
                        Coral coral, Candle candle, PARTsButtonBoxController partsButtonBoxController) {

                addCommands(new ParallelRaceGroup(
                                new SequentialCommandGroup(
                                                candle.addStateCommand(CandleState.AUTO_ALIGN),
                                                drivetrain.alignCommand(new Pose2d(
                                                                new PARTsUnit(-25, PARTsUnitType.Inch)
                                                                                .to(PARTsUnitType.Meter),
                                                                new PARTsUnit(9, PARTsUnitType.Inch)
                                                                                .to(PARTsUnitType.Meter),
                                                                new Rotation2d())),
                                                new WaitCommand(.1),
                                                new ParallelCommandGroup(drivetrain.alignCommand(holdDistance),
                                                                elevator.elevatorToLevelCommand(level))),
                                // coral.score(),
                                new WaitUntilCommand(partsButtonBoxController.negative3Trigger())),
                                candle.removeStateCommand(CandleState.AUTO_ALIGN));

        }
}
