package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.IPARTsSubsystem;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.PARTsUnit;
import frc.robot.util.PARTsUnit.PARTsUnitType;

public class PARTsDrivetrain extends CommandSwerveDrivetrain implements IPARTsSubsystem {
    /*-------------------------------- Private instance variables ---------------------------------*/
    private SwerveDrivePoseEstimator m_poseEstimator;

    private SwerveModule frontRightModule;
    private SwerveModule frontLeftModule;
    private SwerveModule backRightModule;
    private SwerveModule backLeftModule;

    private boolean doRejectUpdate = false;

    public PARTsDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);

        initializePoseEstimator();
        sendToDashboard();
    }

    public PARTsDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);

        initializePoseEstimator();
        sendToDashboard();
    }

    public PARTsDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                modules);

        initializePoseEstimator();
        sendToDashboard();
    }

    /*-------------------------------- Generic Subsystem Functions --------------------------------*/
    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub
        //throw new UnsupportedOperationException("Unimplemented method 'outputTelemetry'");
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        //throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        //throw new UnsupportedOperationException("Unimplemented method 'reset'");
    }

    @Override
    public void log() {
        // TODO Auto-generated method stub
        //throw new UnsupportedOperationException("Unimplemented method 'log'");
    }

    private void initializePoseEstimator() {

        frontLeftModule = getModule(0);
        frontRightModule = getModule(1);
        backLeftModule = getModule(2);
        backRightModule = getModule(3);

        m_poseEstimator = new SwerveDrivePoseEstimator(
                getKinematics(),
                getRotation3d().toRotation2d(),
                new SwerveModulePosition[] {
                        frontLeftModule.getPosition(doRejectUpdate),
                        frontRightModule.getPosition(doRejectUpdate),
                        backLeftModule.getPosition(doRejectUpdate),
                        backRightModule.getPosition(doRejectUpdate)

                },
                new Pose2d(),
                VecBuilder.fill(0.05, 0.05, new PARTsUnit(5, PARTsUnitType.Angle).to(PARTsUnitType.Radian)),
                VecBuilder.fill(0.5, 0.5, new PARTsUnit(30, PARTsUnitType.Angle).to(PARTsUnitType.Radian)));

    }

    /*---------------------------------- Custom Public Functions ----------------------------------*/
    public void updatePoseEstimator() {
        m_poseEstimator.update(
                getPigeon2().getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeftModule.getPosition(doRejectUpdate),
                        frontRightModule.getPosition(doRejectUpdate),
                        backLeftModule.getPosition(doRejectUpdate),
                        backRightModule.getPosition(doRejectUpdate)
                });
    }

    public Rotation2d getEstimatedRotation2d() {
        LimelightHelpers.SetRobotOrientation("", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0,
                0, 0,
                0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");

        if (mt2 != null) {
            if (Math.abs(getPigeon2().getRate()) > 720) // if our angular velocity is greater than 720 degrees per
                                                        // second, ignore vision updates
            {
                doRejectUpdate = true;
            }
            if (mt2.tagCount == 0) {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate) {
                m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                m_poseEstimator.addVisionMeasurement(
                        mt2.pose,
                        mt2.timestampSeconds);
            }
        }

        return mt2 != null ? m_poseEstimator.getEstimatedPosition().getRotation() : null;
    }

    public Pose2d getEstimatedPose() {
        LimelightHelpers.SetRobotOrientation("", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0,
                0, 0,
                0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");

        if (mt2 != null) {
            if (Math.abs(getPigeon2().getRate()) > 720) // if our angular velocity is greater than 720 degrees per
                                                        // second, ignore vision updates
            {
                doRejectUpdate = true;
            }
            if (mt2.tagCount == 0) {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate) {
                m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                m_poseEstimator.addVisionMeasurement(
                        mt2.pose,
                        mt2.timestampSeconds);
            }
        }

        return m_poseEstimator.getEstimatedPosition();
    }

    /*---------------------------------- Custom Private Functions ---------------------------------*/
    private void sendToDashboard() {
        SendableRegistry.addLW(this, getName());

        SmartDashboard.putData("Swerve Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle",
                        () -> frontLeftModule.getCurrentState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity",
                        () -> frontLeftModule.getCurrentState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Front Right Angle",
                        () -> frontRightModule.getCurrentState().angle.getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity",
                        () -> frontRightModule.getCurrentState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Left Angle", () -> backLeftModule.getCurrentState().angle.getRadians(),
                        null);
                builder.addDoubleProperty("Back Left Velocity",
                        () -> backLeftModule.getCurrentState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Right Angle",
                        () -> backRightModule.getCurrentState().angle.getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity",
                        () -> backRightModule.getCurrentState().speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Angle",
                        () -> new PARTsUnit(getPigeon2().getYaw().getValueAsDouble(), PARTsUnitType.Angle)
                                .to(PARTsUnitType.Radian),
                        null);
            }
        });
    }

    /*---------------------------------- Interface Functions ----------------------------------*/
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Subsystem");

        builder.addBooleanProperty(".hasDefault", () -> getDefaultCommand() != null, null);
        builder.addStringProperty(
                ".default",
                () -> getDefaultCommand() != null ? getDefaultCommand().getName() : "none",
                null);
        builder.addBooleanProperty(".hasCommand", () -> getCurrentCommand() != null, null);
        builder.addStringProperty(
                ".command",
                () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none",
                null);
    }

}
