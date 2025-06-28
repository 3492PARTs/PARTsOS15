// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Field;
import frc.robot.Cameras.CameraName;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.PARTsDrivetrain;
import frc.robot.util.PARTsUnit.PARTsUnitType;

/** Add your docs here. */
public class Reef {
    private static Pose2d targetPose2d;

    public static Command alignToVisibleTag(boolean rightSide, PARTsDrivetrain drivetrain) {
        Command c = new ConditionalCommand(Commands.runOnce(() -> {
            int tagID = LimelightVision.getVisibleTagId(CameraName.FRONT_CAMERA.getCameraName());
            System.out.println(tagID);
            targetPose2d = Field.getTag(tagID)
                    .getLocation().toPose2d();
            targetPose2d = targetPose2d
                    .transformBy(new Transform2d(Constants.Robot.frontRobotVisionOffset.to(PARTsUnitType.Meter),
                            (rightSide ? 1 : -1) * Constants.Drivetrain.poleDistanceOffset.to(PARTsUnitType.Meter),
                            new Rotation2d(PARTsUnit.DegreesToRadians.apply(180.0))));
            System.out.println(targetPose2d);
        }).andThen(drivetrain.alignCommand(()-> targetPose2d)), new WaitCommand(0), () -> {
            return LimelightVision.cameraSeesTag(CameraName.FRONT_CAMERA.getCameraName());
        });
        c.setName("alignToVisibleTag");
        return c;
    }
}
