// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Cameras.CameraName;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.PARTsDrivetrain;
import frc.robot.subsystems.Candle.CandleState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.util.PARTsUnit.PARTsUnitType;

/** Add your docs here. */
public class Reef {
    private static Pose2d targetPose2d;

    public static Command alignToVisibleTag(boolean rightSide, PARTsDrivetrain drivetrain, Elevator elevator,
            ElevatorState elevatorState, Coral coral, BooleanSupplier escapeBoolean, Candle candle) {
        Command c = candle.addStateCommand(CandleState.AUTO_ALIGN)
                .andThen(new ParallelDeadlineGroup(new WaitUntilCommand(escapeBoolean),
                        new ConditionalCommand(Commands.runOnce(() -> {
                            int tagID = LimelightVision.getVisibleTagId(CameraName.FRONT_CAMERA.getCameraName());
                            targetPose2d = Field.getTag(tagID)
                                    .getLocation().toPose2d();
                            targetPose2d = targetPose2d
                                    .transformBy(new Transform2d(
                                            (elevatorState == ElevatorState.L4
                                                    ? Constants.Robot.frontRobotVisionL4Offset.to(PARTsUnitType.Meter)
                                                    : Constants.Robot.frontRobotVisionOffset.to(PARTsUnitType.Meter)),
                                            (rightSide ? 1 : -1)
                                                    * Constants.Drivetrain.poleDistanceOffset.to(PARTsUnitType.Meter),
                                            new Rotation2d(PARTsUnit.DegreesToRadians.apply(180.0))));
                        }).andThen(drivetrain.alignCommand(() -> targetPose2d))
                                .andThen(elevator.elevatorToLevelCommand(elevatorState))
                                .andThen(coral.scoreCommand()).andThen(new WaitCommand(0.25))
                                .andThen(elevator.goToElevatorStow()),
                                new WaitCommand(0), () -> {
                                    return LimelightVision.cameraSeesTag(CameraName.FRONT_CAMERA.getCameraName());
                                })))
                .finallyDo(() -> {
                    candle.removeState(CandleState.AUTO_ALIGN);
                });
        c.setName("alignToVisibleTag");
        return c;
    }
}
