// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.cmds.coral;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.PARTsDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Candle.CandleState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAlignScoreCoral extends SequentialCommandGroup {

        /** Creates a new ScoreCoral. */
        public AutoAlignScoreCoral(Pose2d holdDistance, ElevatorState level, PARTsDrivetrain drivetrain, Elevator elevator,
                        Coral coral, Candle candle, Vision vision) {

                addCommands(candle.addStateCommand(CandleState.AUTO_ALIGN),
                                new ParallelCommandGroup(
                                        drivetrain.alignCommand(holdDistance,vision),
                                        elevator.elevatorToLevelCommand(level)),
                                new WaitCommand(.2),
                                coral.autoScore(),
                                new WaitCommand(.2),
                                elevator.elevatorToLevelCommand(ElevatorState.STOW),
                                candle.removeStateCommand(CandleState.AUTO_ALIGN));
                                
}}
