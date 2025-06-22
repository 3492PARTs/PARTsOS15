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
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class HolonomicController {
    private ProfiledPIDController xController;
    private ProfiledPIDController yController;
    private ProfiledPIDController angleController;

    public HolonomicController(ProfiledPIDController xController, ProfiledPIDController yController, ProfiledPIDController angleController) {
        this.xController = xController;
        this.yController = yController;
        this.angleController = angleController;
    }

    public ChassisSpeeds calculate(Pose2d measurement, Rotation2d angle) {
        
        return ChassisSpeeds.fromFieldRelativeSpeeds(
            xController.calculate(measurement.getX()) *-1,
            yController.calculate(measurement.getY()) * 0,
            angleController.calculate(measurement.getRotation().getRadians())*0,
                angle);
    }

    public void setGoal(Pose2d goal) {
        xController.setGoal(goal.getX());
        yController.setGoal(goal.getY());
        angleController.setGoal(goal.getRotation().getRadians());
    }

    public void reset(Pose2d goal) {
        xController.reset(goal.getX());
        yController.reset(goal.getY());
        angleController.reset(goal.getRotation().getRadians());
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

    public State getYGoal() {
        return yController.getGoal();
    }
    
    public double getYError() {
        return yController.getPositionError();
    }

    public double getXError() {
        return xController.getPositionError();
    }

    public State getYSetPoint() {
        return yController.getSetpoint();
    }
}