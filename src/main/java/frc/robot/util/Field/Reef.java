// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.Field;

import java.util.Arrays;
import java.util.HashSet;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.constants.CameraConstants.CameraName;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.states.CandleState;
import frc.robot.states.ElevatorState;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Coral.Coral;
import frc.robot.subsystems.Drivetrain.PARTsDrivetrain;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.util.PARTs.Classes.PARTsCommandUtils;
import frc.robot.util.PARTs.Classes.PARTsUnit;
import frc.robot.util.PARTs.Classes.PARTsUnit.PARTsUnitType;

/** Add your docs here. */
public class Reef {
        private static Pose2d targetPose2d;

        public static Command commandAlignAndScoreToVisibleTag(boolean rightSide, PARTsDrivetrain drivetrain,
                        Elevator elevator,
                        ElevatorState elevatorState, Coral coral, BooleanSupplier escapeBoolean, Candle candle,
                        BooleanSupplier visionActiveBooleanSupplier) {
                return PARTsCommandUtils.setCommandName("commandAlignAndScoreToVisibleTag",
                                Commands.either(
                                                candle.commandAddState(CandleState.AUTO_ALIGN)
                                                                .andThen(new ParallelDeadlineGroup(
                                                                                new WaitUntilCommand(escapeBoolean),
                                                                                new ConditionalCommand(
                                                                                                Commands.runOnce(() -> {
                                                                                                        int tagID = LimelightVision
                                                                                                                        .getVisibleTagId(
                                                                                                                                        CameraName.FRONT_CAMERA
                                                                                                                                                        .getCameraName());
                                                                                                        targetPose2d = Field
                                                                                                                        .getTag(tagID)
                                                                                                                        .getLocation()
                                                                                                                        .toPose2d();
                                                                                                        targetPose2d = targetPose2d
                                                                                                                        .transformBy(new Transform2d(
                                                                                                                                        (elevatorState == ElevatorState.L4
                                                                                                                                                        ? RobotConstants.ROBOT_VISION_L4_OFFSET
                                                                                                                                                                        .to(PARTsUnitType.Meter)
                                                                                                                                                        : RobotConstants.ROBOT_VISION_OFFSET
                                                                                                                                                                        .to(PARTsUnitType.Meter)),
                                                                                                                                        (rightSide ? 1
                                                                                                                                                        : -1)
                                                                                                                                                        * DrivetrainConstants.POLE_DISTANCE_OFFSET
                                                                                                                                                                        .to(PARTsUnitType.Meter),
                                                                                                                                        new Rotation2d(PARTsUnit.DegreesToRadians
                                                                                                                                                        .apply(180.0))));
                                                                                                }).andThen(drivetrain
                                                                                                                .commandAlign(() -> targetPose2d))
                                                                                                                .andThen(elevator
                                                                                                                                .commandToLevel(
                                                                                                                                                elevatorState))
                                                                                                                .andThen(coral.commandScore(
                                                                                                                                elevator.getStateSupplier()))
                                                                                                                .andThen(new WaitCommand(
                                                                                                                                0.25))
                                                                                                                .andThen(elevator
                                                                                                                                .commandStow()),
                                                                                                new WaitCommand(0),
                                                                                                () -> {
                                                                                                        return LimelightVision
                                                                                                                        .cameraSeesTag(
                                                                                                                                        CameraName.FRONT_CAMERA
                                                                                                                                                        .getCameraName());
                                                                                                })))
                                                                .finallyDo(() -> {
                                                                        candle.removeState(CandleState.AUTO_ALIGN);
                                                                }),
                                                elevator.commandToLevel(elevatorState),
                                                visionActiveBooleanSupplier));
        }

        public static Command commandIntakeScoreIntake(PARTsDrivetrain drivetrain, Coral coral, Elevator elevator) {
                Pose2d feederStationGoal = Field.getTag(12).getLocation().toPose2d();
                feederStationGoal = feederStationGoal.transformBy(
                                new Transform2d(RobotConstants.ROBOT_VISION_OFFSET.to(PARTsUnitType.Meter),
                                                0,
                                                new Rotation2d(PARTsUnit.DegreesToRadians.apply(0.0))));

                Pose2d feederStation1M = feederStationGoal.transformBy(new Transform2d(1, 0, new Rotation2d()));

                Pose2d reefGoal = Field.getTag(18).getLocation().toPose2d();
                reefGoal = reefGoal.transformBy(
                                new Transform2d(RobotConstants.ROBOT_VISION_OFFSET.to(PARTsUnitType.Meter),
                                                0,
                                                new Rotation2d(PARTsUnit.DegreesToRadians.apply(180.0))));

                Pose2d reefGoal1M = reefGoal.transformBy(new Transform2d(-1, 0, new Rotation2d()));

                Command c = new SequentialCommandGroup(drivetrain.commandPathFindToPose(feederStation1M),
                                drivetrain.commandAlign(feederStationGoal), coral.commandAutoIntake(),
                                drivetrain.commandPathFindToPose(reefGoal1M), drivetrain.commandAlign(reefGoal),
                                elevator.commandToLevel(ElevatorState.L2),
                                coral.commandScore(elevator.getStateSupplier()),
                                new WaitCommand(0.25),
                                elevator.commandToLevel(ElevatorState.STOW));
                c.setName("commandIntakeScoreIntake");
                return c;
        }

        public static Command alignToVisibleTagSideStop(boolean rightSide, PARTsDrivetrain drivetrain,
                        Elevator elevator,
                        ElevatorState elevatorState, Coral coral, BooleanSupplier escapeBoolean, Candle candle) {
                Command c = candle.commandAddState(CandleState.AUTO_ALIGN)
                                .andThen(new ParallelDeadlineGroup(new WaitUntilCommand(escapeBoolean),
                                                new ConditionalCommand(Commands.runOnce(() -> {
                                                        int tagID = LimelightVision.getVisibleTagId(
                                                                        CameraName.FRONT_CAMERA.getCameraName());
                                                        targetPose2d = Field.getTag(tagID)
                                                                        .getLocation().toPose2d();
                                                        targetPose2d = targetPose2d
                                                                        .transformBy(new Transform2d(
                                                                                        (elevatorState == ElevatorState.L4
                                                                                                        ? RobotConstants.ROBOT_VISION_L4_OFFSET
                                                                                                                        .to(PARTsUnitType.Meter)
                                                                                                        : RobotConstants.ROBOT_VISION_OFFSET
                                                                                                                        .to(PARTsUnitType.Meter)),
                                                                                        (rightSide ? 1 : -1)
                                                                                                        * DrivetrainConstants.POLE_DISTANCE_OFFSET
                                                                                                                        .to(PARTsUnitType.Meter),
                                                                                        new Rotation2d(PARTsUnit.DegreesToRadians
                                                                                                        .apply(180.0))));
                                                }).andThen(drivetrain.commandAlign(() -> targetPose2d))
                                                                .andThen(elevator
                                                                                .commandToLevel(elevatorState)),
                                                                new WaitCommand(0), () -> {
                                                                        return LimelightVision.cameraSeesTag(
                                                                                        CameraName.FRONT_CAMERA
                                                                                                        .getCameraName());
                                                                })))
                                .finallyDo(() -> {
                                        candle.removeState(CandleState.AUTO_ALIGN);
                                });
                c.setName("alignToVisibleTagStop");
                return c;
        }
}
