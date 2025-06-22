package frc.robot.cmds.coral;

/************************ PROJECT MARY *************************/
/* Copyright (c) 2025 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Field;
import frc.robot.HolonomicController;
import frc.robot.Robot;
import frc.robot.subsystems.PARTsDrivetrain;
import frc.robot.util.PARTsUnit.PARTsUnitType;

import java.util.Vector;
import java.util.function.Supplier;

public class SwerveDrivePIDToPose extends Command {

    private final PARTsDrivetrain swerve;

    private FieldObject2d computedPose;

    private final HolonomicController controller;
    private final Supplier<Pose2d> targetPose;

    private double maxVelocity;
    private double maxAcceleration;

    private boolean isMotionProfiled;

    private final Supplier<Boolean> isAligned;
    private final Supplier<Double> velocityError;

    private final FieldObject2d targetPose2d;

    private Number xTolerance;
    private Number yTolerance;
    private Number thetaTolerance;
    private Number maxVelocityWhenAligned;

    private Supplier<Translation2d> translationSetpoint;

    private Supplier<Boolean> canEnd;

    private FieldObject2d targetObject2d;

    public SwerveDrivePIDToPose(Pose2d targetPose, PARTsDrivetrain swerve) {
        this(() -> targetPose, swerve);
    }

    public SwerveDrivePIDToPose(Supplier<Pose2d> targetPose, PARTsDrivetrain swerve) {
        targetObject2d = Field.FIELD2D.getObject("Target Pose");
        //computedPose = Field.FIELD2D.getObject("Computed Pose");
        this.swerve = swerve;

        controller = new HolonomicController(
                new ProfiledPIDController(Constants.Drivetrain.RANGE_X_P, Constants.Drivetrain.RANGE_I,
                        Constants.Drivetrain.RANGE_D,
                        new TrapezoidProfile.Constraints(Constants.Drivetrain.MAX_RANGE_VELOCITY,
                                Constants.Drivetrain.MAX_RANGE_ACCELERATION)),
                new ProfiledPIDController(Constants.Drivetrain.RANGE_Y_P, Constants.Drivetrain.RANGE_I,
                        Constants.Drivetrain.RANGE_D,
                        new TrapezoidProfile.Constraints(Constants.Drivetrain.MAX_RANGE_VELOCITY,
                                Constants.Drivetrain.MAX_RANGE_ACCELERATION)),
                new ProfiledPIDController(Constants.Drivetrain.THETA_P, Constants.Drivetrain.THETA_I,
                        Constants.Drivetrain.THETA_D,
                        new TrapezoidProfile.Constraints(Constants.Drivetrain.MAX_AIM_VELOCITY,
                                Constants.Drivetrain.MAX_AIM_ACCELERATION)));

        maxVelocity = Constants.Drivetrain.MAX_AIM_VELOCITY;
        maxAcceleration = Constants.Drivetrain.MAX_AIM_VELOCITY;
        isMotionProfiled = false;
        translationSetpoint = getNewTranslationSetpointGenerator();

        this.targetPose = targetPose;

        targetPose2d = Field.FIELD2D.getObject("Target Pose");
        isAligned = this::isAligned;
        /*
         * isAligned = BStream.create(this::isAligned)
         * .filtered(new
         * BDebounceRC.Both(Settings.Swerve.Alignment.Tolerances.ALIGNMENT_DEBOUNCE));
         */

        velocityError = () -> new Translation2d(controller.getError().vxMetersPerSecond,
                controller.getError().vyMetersPerSecond).getNorm();

        xTolerance = Constants.Drivetrain.xRControllerTolerance.to(PARTsUnitType.Meter);
        yTolerance = Constants.Drivetrain.yRControllerTolerance.to(PARTsUnitType.Meter);
        thetaTolerance = Constants.Drivetrain.thetaControllerTolerance.to(PARTsUnitType.Radian);
        maxVelocityWhenAligned = 0;

        canEnd = () -> true;

        addRequirements(swerve);
    }

    public SwerveDrivePIDToPose withTolerance(double x, double y, Rotation2d theta) {
        xTolerance = x;
        yTolerance = y;
        thetaTolerance = theta.getRadians();
        return this;
    }

    public SwerveDrivePIDToPose withTranslationalConstraints(double maxVelocity, double maxAcceleration) {
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        return this;
    }

    public SwerveDrivePIDToPose withoutMotionProfile() {
        this.isMotionProfiled = false;
        return this;
    }

    public SwerveDrivePIDToPose withCanEnd(Supplier<Boolean> canEnd) {
        this.canEnd = canEnd;
        return this;
    }

    // the VStream needs to be recreated everytime the command is scheduled to allow
    // the target tranlation to jump to the start of the path
    private Supplier<Translation2d> getNewTranslationSetpointGenerator() {
        if (!isMotionProfiled) {
            return () -> (targetPose.get().getTranslation());
        } else {
            throw new UnsupportedOperationException("Unimplemented method 'log'");
            /*
             * return VStream.create(() -> new Vector2D(targetPose.get().getTranslation()))
             * .filtered(new TranslationMotionProfileIan(
             * this.maxVelocity,
             * this.maxAcceleration,
             * new Vector2D(swerve.getPose().getTranslation()),
             * Vector2D.kOrigin));
             */
        }
    }

    @Override
    public void initialize() {
        translationSetpoint = getNewTranslationSetpointGenerator();
    }

    private boolean isAlignedX() {
        return Math.abs(targetPose.get().getX() - swerve.getPose().getX()) < xTolerance.doubleValue();
    }

    private boolean isAlignedY() {
        return Math.abs(targetPose.get().getY() - swerve.getPose().getY()) < yTolerance.doubleValue();
    }

    private boolean isAlignedTheta() {
        return Math.abs(targetPose.get().getRotation().minus(swerve.getPose().getRotation())
                .getRadians()) < thetaTolerance.doubleValue();
    }

    private boolean isAligned() {
        return isAlignedX() && isAlignedY() && isAlignedTheta()
                && velocityError.get() < maxVelocityWhenAligned.doubleValue();
    }

    @Override
    public void execute() {
        // targetPose2d.setPose(Robot.isBlue() ? targetPose.get() :
        // Field.transformToOppositeAlliance(targetPose.get()));

        targetObject2d.setPose(targetPose.get());
        Pose2d storedPose = targetPose.get(); // new Pose2d(translationSetpoint.get(), targetPose.get().getRotation());
        //computedPose.setPose(storedPose);

        ChassisSpeeds speeds = controller.update(storedPose,
                swerve.getPose(), swerve.getPose().getRotation());

        swerve.setChassisSpeeds(speeds);

        SmartDashboard.putNumber("Alignment/Target x", targetPose.get().getX());
        SmartDashboard.putNumber("Alignment/Target y", targetPose.get().getY());
        SmartDashboard.putNumber("Alignment/Target angle", targetPose.get().getRotation().getDegrees());

        SmartDashboard.putNumber("Alignment/Target Velocity Robot Relative X (m per s)", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Alignment/Target Velocity Robot Relative Y (m per s)", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Alignment/Target Angular Velocity (rad per s)", speeds.omegaRadiansPerSecond);

        SmartDashboard.putBoolean("Alignment/Is Aligned", isAligned());
        SmartDashboard.putBoolean("Alignment/Is Aligned X", isAlignedX());
        SmartDashboard.putBoolean("Alignment/Is Aligned Y", isAlignedY());
        SmartDashboard.putBoolean("Alignment/Is Aligned Theta", isAlignedTheta());
    }

    @Override
    public boolean isFinished() {
        return isAlignedX() && canEnd.get();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(swerve.getFieldCentricSwerveRequest()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0));
    }
}