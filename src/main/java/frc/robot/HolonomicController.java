package frc.robot;


import edu.wpi.first.math.controller.ProfiledPIDController;

/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class HolonomicController {
    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController angleController;

    public HolonomicController(ProfiledPIDController xController, ProfiledPIDController yController, ProfiledPIDController angleController) {
        this.xController = xController;
        this.yController = yController;
        this.angleController = angleController;
    }

    public ChassisSpeeds update(Pose2d measurement, Pose2d goal, Rotation2d angle) {
        
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            xController.calculate(measurement.getX(), goal.getX()) *-1 *0,
            yController.calculate(measurement.getY(), goal.getY()),
            angleController.calculate(goal.getRotation().getRadians(), measurement.getRotation().getRadians())*0,
                angle);
    }

    public ChassisSpeeds getError() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xController.getPositionError(),
                yController.getPositionError(),
                angleController.getVelocityError(),
                new Rotation2d(angleController.getPositionError()));
    }

    public boolean isDone(double xToleranceMeters, double yToleranceMeters, double angleToleranceDegrees) {
        xController.setTolerance(xToleranceMeters);
        yController.setTolerance(yToleranceMeters);
        angleController.setTolerance(angleToleranceDegrees);
        return xController.atGoal()
                && yController.atGoal()
                && angleController.atGoal();
    }
}