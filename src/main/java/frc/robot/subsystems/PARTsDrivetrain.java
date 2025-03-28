package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.AprilTagData;
import frc.robot.util.IPARTsSubsystem;
import frc.robot.util.PARTsLogger;
import frc.robot.util.PARTsNT;
import frc.robot.util.PARTsUnit;
import frc.robot.util.PARTsUnit.PARTsUnitType;

public class PARTsDrivetrain extends CommandSwerveDrivetrain implements IPARTsSubsystem {
        /*-------------------------------- Private instance variables ---------------------------------*/
        private SwerveDrivePoseEstimator m_poseEstimator;
        private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

        private SwerveModule<TalonFX, TalonFX, CANcoder> frontRightModule;
        private SwerveModule<TalonFX, TalonFX, CANcoder> frontLeftModule;
        private SwerveModule<TalonFX, TalonFX, CANcoder> backRightModule;
        private SwerveModule<TalonFX, TalonFX, CANcoder> backLeftModule;

        private boolean doRejectUpdate = false;

        private PARTsNT partsNT;
        private PARTsLogger partsLogger;

        // Vision Variables
        private Vision m_vision;
        private SwerveRequest.RobotCentric alignRequest;

        private ProfiledPIDController thetaController;
        private ProfiledPIDController xRangeController;
        private ProfiledPIDController yRangeController;

        // Robot poses.
        private Pose2d initialLLPose2d;
        private Pose3d currentEstimatedRobotPose3d;
        private Pose3d currentVisionPose3d;
        // private double initialRobotAngleRad;

        private Pose2d initialPose2d;
        private double tagID = -1;
        // double skewVal;

        //LimelightHelpers.PoseEstimate mt2 = null;

        public PARTsDrivetrain(
                        SwerveDrivetrainConstants drivetrainConstants,
                        SwerveModuleConstants<?, ?, ?>... modules) {
                super(drivetrainConstants, modules);

                initialize();
        }

        public PARTsDrivetrain(
                        SwerveDrivetrainConstants drivetrainConstants,
                        double odometryUpdateFrequency,
                        SwerveModuleConstants<?, ?, ?>... modules) {
                super(drivetrainConstants, odometryUpdateFrequency, modules);

                initialize();
        }

        public PARTsDrivetrain(
                        SwerveDrivetrainConstants drivetrainConstants,
                        double odometryUpdateFrequency,
                        Matrix<N3, N1> odometryStandardDeviation,
                        Matrix<N3, N1> visionStandardDeviation,
                        SwerveModuleConstants<?, ?, ?>... modules) {
                super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                                modules);

                initialize();

        }

        /*-------------------------------- Generic Subsystem Functions --------------------------------*/
        @Override
        public void outputTelemetry() {
                partsNT.setDouble("vision/tagPoseX", new PARTsUnit(currentVisionPose3d.getX(), PARTsUnitType.Meter)
                                .to(PARTsUnitType.Inch));
                partsNT.setDouble("vision/tagPoseY", new PARTsUnit(currentVisionPose3d.getY(), PARTsUnitType.Meter)
                                .to(PARTsUnitType.Inch));
                partsNT.setDouble("vision/tagPoseRot", new PARTsUnit(currentVisionPose3d.getRotation().getAngle(),
                                PARTsUnitType.Radian).to(PARTsUnitType.Angle));

                partsNT.setBoolean("vision/tag", tagID > 0);
                partsNT.setDouble("vision/tagID", tagID);
        }

        @Override
        public void stop() {
                applyRequest(() -> new SwerveRequest.RobotCentric().withVelocityX(0)
                                .withVelocityY(0)
                                .withRotationalRate(0));
        }

        @Override
        public void reset() {
                // TODO Auto-generated method stub
                // throw new UnsupportedOperationException("Unimplemented method 'reset'");
        }

        @Override
        public void log() {
                // TODO Auto-generated method stub
                // throw new UnsupportedOperationException("Unimplemented method 'log'");
        }

        @Override
        public void periodic() {
                super.periodic();

        }

        /*---------------------------------- Custom Public Functions ----------------------------------*/

        public Command alignCommand(Pose2d holdDistance, Vision vision) {
                Command c = new ConditionalCommand(new FunctionalCommand(
                                () -> {
                                        vision.periodic();
                                        initializePoseEstimator();
                                        
                                        resetPoseEstimator(initialPose2d);

                                        // Initialize the aim controller.
                                        thetaController.reset(m_poseEstimator.getEstimatedPosition().getRotation()
                                                        .getRadians());

                                        thetaController.setGoal(holdDistance.getRotation().getRadians()); // tx=0
                                                                                                          // is
                                                                                                          // centered.
                                        thetaController.setTolerance(
                                                        Constants.Drivetrain.thetaControllerTolerance
                                                                        .to(PARTsUnitType.Radian));

                                        // Initialize the x-range controller.
                                        xRangeController.reset(m_poseEstimator.getEstimatedPosition().getX());
                                        xRangeController.setGoal(holdDistance.getX());
                                        xRangeController.setTolerance(Constants.Drivetrain.xRControllerTolerance
                                                        .to(PARTsUnitType.Meter));

                                        // Initialize the y-range controller.
                                        yRangeController.reset(m_poseEstimator.getEstimatedPosition().getY()); // Center
                                                                                                               // to
                                                                                                               // target.
                                        yRangeController.setGoal(holdDistance.getY()); // Center to target.
                                        yRangeController.setTolerance(Constants.Drivetrain.yRControllerTolerance
                                                        .to(PARTsUnitType.Meter));

                                        alignCommandInitTelemetry(holdDistance);
                                },
                                () -> {
                                        updatePoseEstimator();
                                        currentEstimatedRobotPose3d = new Pose3d(
                                                        m_poseEstimator.getEstimatedPosition());

                                        Rotation2d thetaOutput = new Rotation2d(
                                                        thetaController.calculate(
                                                                        currentEstimatedRobotPose3d.getRotation()
                                                                                        .toRotation2d().getRadians()));

                                        Pose2d rangeOutput = new Pose2d(
                                                        xRangeController.calculate(currentEstimatedRobotPose3d.getX(),
                                                                        holdDistance.getX()),
                                                        yRangeController.calculate(currentEstimatedRobotPose3d.getY(),
                                                                        holdDistance.getY()),
                                                        null);

                                        // Get dist. from drivetrain.

                                        Translation2d translation = new Translation2d(rangeOutput.getX(),
                                                        rangeOutput.getY());

                                        super.setControl(alignRequest
                                                        .withVelocityX(translation.getX())
                                                        .withVelocityY(translation.getY())
                                                        .withRotationalRate(thetaOutput.getRadians()));

                                        alignCommandExecuteTelemetry(thetaOutput, rangeOutput);
                                },
                                (Boolean b) -> {
                                        super.setControl(alignRequest
                                                        .withVelocityX(0)
                                                        .withVelocityY(0)
                                                        .withRotationalRate(0));
                                },
                                () -> ((xRangeController.atGoal() &&
                                                yRangeController.atGoal() &&
                                                thetaController.atGoal())),
                                this), new WaitCommand(0), () -> tagID > 0);
                c.setName("align");
                return c;
        }

        /*---------------------------------- Custom Private Functions ---------------------------------*/
        private void alignCommandInitTelemetry(Pose2d holdDist) {
                partsNT.setDouble("align/startMS", System.currentTimeMillis());

                partsLogger.logDouble("align/llPoseX", initialLLPose2d.getX());
                partsLogger.logDouble("align/llPoseY", initialLLPose2d.getY());
                partsLogger.logDouble("align/llPoseRot",
                                new PARTsUnit(initialLLPose2d.getRotation().getRadians(), PARTsUnitType.Radian)
                                                .to(PARTsUnitType.Angle));

                partsNT.setDouble("align/llPoseX", initialLLPose2d.getX());
                partsNT.setDouble("align/llPoseY", initialLLPose2d.getY());
                partsNT.setDouble("align/llPoseRot",
                                new PARTsUnit(initialLLPose2d.getRotation().getRadians(), PARTsUnitType.Radian)
                                                .to(PARTsUnitType.Angle));

                                                partsNT.setDouble("align/holdDistX", new PARTsUnit(holdDist.getX(), PARTsUnitType.Meter)
                                                .to(PARTsUnitType.Inch));
                partsNT.setDouble("align/holdDistY", new PARTsUnit(holdDist.getY(), PARTsUnitType.Meter)
                .to(PARTsUnitType.Inch));
                partsNT.setDouble("align/holdDistRot",
                                new PARTsUnit(holdDist.getRotation().getRadians(), PARTsUnitType.Radian)
                                                .to(PARTsUnitType.Angle));

                partsLogger.logDouble("align/thetaControllerSetpoint",
                                thetaController.getSetpoint().position);
                partsNT.setDouble("align/thetaControllerSetpoint",
                                thetaController.getSetpoint().position);

                
        }

        private void alignCommandExecuteTelemetry(Rotation2d thetaOutput, Pose2d rangeOutput) {
                /* 
                partsLogger.logDouble("align/mt2PoseX",
                                mt2 != null ? new PARTsUnit(mt2.pose.getX(), PARTsUnitType.Meter)
                                                .to(PARTsUnitType.Inch) : -9999);
                partsLogger.logDouble("align/mt2PoseY",
                                mt2 != null ? new PARTsUnit(mt2.pose.getY(), PARTsUnitType.Meter)
                                                .to(PARTsUnitType.Inch) : -9999);
                partsLogger.logDouble("align/mt2PoseRot",
                                mt2 != null ? new PARTsUnit(mt2.pose.getRotation().getRadians(),
                                                PARTsUnitType.Radian).to(PARTsUnitType.Angle) : -9999);
                
                partsNT.setDouble("align/mt2PoseX", mt2 != null ? new PARTsUnit(mt2.pose.getX(), PARTsUnitType.Meter)
                                .to(PARTsUnitType.Inch) : -9999);
                partsNT.setDouble("align/mt2PoseY", mt2 != null ? new PARTsUnit(mt2.pose.getY(), PARTsUnitType.Meter)
                                .to(PARTsUnitType.Inch) : -9999);
                partsNT.setDouble("align/mt2PoseRot", mt2 != null ? new PARTsUnit(mt2.pose.getRotation().getRadians(),
                                PARTsUnitType.Radian).to(PARTsUnitType.Angle) : -9999);*/

                partsLogger.logDouble("align/estPoseX",
                                new PARTsUnit(m_poseEstimator.getEstimatedPosition().getX(), PARTsUnitType.Meter)
                                                .to(PARTsUnitType.Inch));
                partsLogger.logDouble("align/estPoseY",
                                new PARTsUnit(m_poseEstimator.getEstimatedPosition().getY(), PARTsUnitType.Meter)
                                                .to(PARTsUnitType.Inch));
                partsLogger.logDouble("align/estPoseRot",
                                new PARTsUnit(m_poseEstimator.getEstimatedPosition().getRotation().getRadians(),
                                                PARTsUnitType.Radian).to(PARTsUnitType.Angle));

                partsNT.setDouble("align/estPoseX",
                                new PARTsUnit(m_poseEstimator.getEstimatedPosition().getX(), PARTsUnitType.Meter)
                                                .to(PARTsUnitType.Inch));
                partsNT.setDouble("align/estPoseY",
                                new PARTsUnit(m_poseEstimator.getEstimatedPosition().getY(), PARTsUnitType.Meter)
                                                .to(PARTsUnitType.Inch));
                partsNT.setDouble("align/estPoseRot",
                                new PARTsUnit(m_poseEstimator.getEstimatedPosition().getRotation().getRadians(),
                                                PARTsUnitType.Radian).to(PARTsUnitType.Angle));

                partsLogger.logDouble("align/rPoseX",
                                new PARTsUnit(currentEstimatedRobotPose3d.getX(), PARTsUnitType.Meter)
                                                .to(PARTsUnitType.Inch));
                partsLogger.logDouble("align/rPoseY",
                                new PARTsUnit(currentEstimatedRobotPose3d.getY(), PARTsUnitType.Meter)
                                                .to(PARTsUnitType.Inch));
                partsLogger.logDouble("align/rPoseRot",
                                new PARTsUnit(currentEstimatedRobotPose3d.getRotation().getAngle(),
                                                PARTsUnitType.Radian).to(PARTsUnitType.Angle));

                partsNT.setDouble("align/rPoseX",
                                new PARTsUnit(currentEstimatedRobotPose3d.getX(), PARTsUnitType.Meter)
                                                .to(PARTsUnitType.Inch));
                partsNT.setDouble("align/rPoseY",
                                new PARTsUnit(currentEstimatedRobotPose3d.getY(), PARTsUnitType.Meter)
                                                .to(PARTsUnitType.Inch));
                partsNT.setDouble("align/rPoseRot",
                                new PARTsUnit(currentEstimatedRobotPose3d.getRotation().getAngle(),
                                                PARTsUnitType.Radian).to(PARTsUnitType.Angle));

                partsLogger.logDouble("align/Output/thetaController", thetaOutput.getDegrees());
                partsLogger.logDouble("align/Output/rangeControllerX", rangeOutput.getX());
                partsLogger.logDouble("align/Output/rangeControllerY", rangeOutput.getY());

                partsNT.setDouble("align/Output/thetaController", thetaOutput.getDegrees());
                partsNT.setDouble("align/Output/rangeControllerX", rangeOutput.getX());
                partsNT.setDouble("align/Output/rangeControllerY", rangeOutput.getY());

                partsLogger.logBoolean("align/Goal/thetaController", thetaController.atGoal());
                partsLogger.logBoolean("align/Goal/rangeControllerX",
                                xRangeController.atGoal());
                partsLogger.logBoolean("align/Goal/rangeControllerYGoal",
                                yRangeController.atGoal());
                partsNT.setDouble("align/Goal/x setpoint", xRangeController.getSetpoint().position);
                partsNT.setDouble("align/Goal/y setpoint", yRangeController.getSetpoint().position);
                partsNT.setDouble("align/Goal/x setpoint", thetaController.getSetpoint().position);

                partsNT.setBoolean("align/Goal/thetaController", thetaController.atGoal());
                partsNT.setBoolean("align/Goal/rangeControllerX", xRangeController.atGoal());
                partsNT.setBoolean("align/Goal/rangeControllerY", yRangeController.atGoal());

                partsLogger.logDouble("align/Output/PosErrorX",
                                xRangeController.getPositionError());
                partsLogger.logDouble("align/Output/PosErrorY",
                                yRangeController.getPositionError());
                partsLogger.logDouble("align/Output/thetaPosError",
                                thetaController.getPositionError());

                partsLogger.logDouble("align/Output/velocityErrorX",
                                xRangeController.getVelocityError());
                partsLogger.logDouble("align/Output/velocityErrorY",
                                yRangeController.getVelocityError());
                partsLogger.logDouble("align/Output/thetaVelocityError",
                                thetaController.getVelocityError());

                partsNT.setDouble("align/Output/PosErrorX",
                                xRangeController.getPositionError());
                partsNT.setDouble("align/Output/PosErrorY",
                                yRangeController.getPositionError());
                partsNT.setDouble("align/Output/thetaPosError",
                                thetaController.getPositionError());

                partsNT.setDouble("align/Output/velocityErrorX",
                                xRangeController.getVelocityError());
                partsNT.setDouble("align/Output/velocityErrorY",
                                yRangeController.getVelocityError());
                partsNT.setDouble("align/Output/thetaVelocityError",
                                thetaController.getVelocityError());
        }

        private void initialize() {
                frontLeftModule = getModule(0);
                frontRightModule = getModule(1);
                backLeftModule = getModule(2);
                backRightModule = getModule(3);
                initializeClasses();
                initializeControllers();
                sendToDashboard();
                configureAutoBuilder();
                AprilTagData.InitAprilTagObjects();
                initializePoseEstimator();
        }

        private void sendToDashboard() {
                SendableRegistry.addLW(this, getName());

                partsNT.putSmartDashboardSendable("Swerve Drive", new Sendable() {
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

                                builder.addDoubleProperty("Back Left Angle",
                                                () -> backLeftModule.getCurrentState().angle.getRadians(),
                                                null);
                                builder.addDoubleProperty("Back Left Velocity",
                                                () -> backLeftModule.getCurrentState().speedMetersPerSecond, null);

                                builder.addDoubleProperty("Back Right Angle",
                                                () -> backRightModule.getCurrentState().angle.getRadians(), null);
                                builder.addDoubleProperty("Back Right Velocity",
                                                () -> backRightModule.getCurrentState().speedMetersPerSecond, null);

                                builder.addDoubleProperty("Robot Angle",
                                                () -> new PARTsUnit(getPigeon2().getYaw().getValueAsDouble(),
                                                                PARTsUnitType.Angle)
                                                                .to(PARTsUnitType.Radian),
                                                null);
                        }
                });
        }

        private void initializePoseEstimator() {
                m_poseEstimator = new SwerveDrivePoseEstimator(
                                super.getKinematics(),
                                super.getPigeon2().getRotation2d(),
                                new SwerveModulePosition[] {
                                                frontLeftModule.getPosition(true),
                                                frontRightModule.getPosition(true),
                                                backLeftModule.getPosition(true),
                                                backRightModule.getPosition(true)

                                },
                                new Pose2d(),
                                VecBuilder.fill(0.05, 0.05,
                                                new PARTsUnit(5, PARTsUnitType.Angle).to(PARTsUnitType.Radian)),
                                VecBuilder.fill(0.5, 0.5,
                                                new PARTsUnit(30, PARTsUnitType.Angle).to(PARTsUnitType.Radian)));

        }

        private void initializeControllers() {
                alignRequest = new SwerveRequest.RobotCentric()
                                .withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
                                .withRotationalDeadband(0.1);

                thetaController = new ProfiledPIDController(Constants.Drivetrain.THETA_P, Constants.Drivetrain.THETA_I,
                                Constants.Drivetrain.THETA_D,
                                new TrapezoidProfile.Constraints(Constants.Drivetrain.MAX_AIM_VELOCITY,
                                                Constants.Drivetrain.MAX_AIM_ACCELERATION));
                thetaController.enableContinuousInput(-Math.PI, Math.PI); // Wrpa from -pi to ip

                xRangeController = new ProfiledPIDController(Constants.Drivetrain.RANGE_X_P,
                                Constants.Drivetrain.RANGE_I,
                                Constants.Drivetrain.RANGE_D,
                                new TrapezoidProfile.Constraints(Constants.Drivetrain.MAX_RANGE_VELOCITY,
                                                Constants.Drivetrain.MAX_RANGE_ACCELERATION));
                yRangeController = new ProfiledPIDController(Constants.Drivetrain.RANGE_Y_P,
                                Constants.Drivetrain.RANGE_I,
                                Constants.Drivetrain.RANGE_D,
                                new TrapezoidProfile.Constraints(Constants.Drivetrain.MAX_RANGE_VELOCITY,
                                                Constants.Drivetrain.MAX_RANGE_ACCELERATION));

        }

        private void initializeClasses() {
                partsNT = new PARTsNT(this);
                partsLogger = new PARTsLogger(this);
        }

        private ChassisSpeeds getChassisSpeeds() {
                return getKinematics().toChassisSpeeds(
                                frontLeftModule.getCurrentState(),
                                frontRightModule.getCurrentState(),
                                backLeftModule.getCurrentState(),
                                backRightModule.getCurrentState());
        }

        private void updatePoseEstimator() {
                m_poseEstimator.update(
                                super.getPigeon2().getRotation2d(),
                                new SwerveModulePosition[] {
                                                frontLeftModule.getPosition(true),
                                                frontRightModule.getPosition(true),
                                                backLeftModule.getPosition(true),
                                                backRightModule.getPosition(true)
                                });
        }

        private void resetPoseEstimator(Pose2d pose) {
                m_poseEstimator.resetPose(pose);
        }

        private void setPoseEstimatorVisionMeasurement() {
                // if our angular velocity is greater than 360 degrees per second, ignore vision
                // updates
                /*if (Math.abs(super.getPigeon2().getRate()) > 360) {
                        doRejectUpdate = true;
                }
                
                if (mt2 == null || mt2.tagCount == 0) {
                        doRejectUpdate = true;
                }
                
                if (!doRejectUpdate) {
                        m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                        m_poseEstimator.addVisionMeasurement(
                                        mt2.pose,
                                        mt2.timestampSeconds);
                }*/
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

        /*---------------------------------- AutoBuilder Functions ----------------------------------*/

        private void configureAutoBuilder() {
                try {
                        var config = RobotConfig.fromGUISettings();
                        AutoBuilder.configure(
                                        () -> getState().Pose, // Supplier of current robot pose
                                        this::resetPose, // Consumer for seeding pose against auto
                                        () -> getState().Speeds, // Supplier of current robot speeds
                                        // Consumer of ChassisSpeeds and feedforwards to drive the robot
                                        (speeds, feedforwards) -> setControl(
                                                        m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                                                        .withWheelForceFeedforwardsX(feedforwards
                                                                                        .robotRelativeForcesXNewtons())
                                                                        .withWheelForceFeedforwardsY(feedforwards
                                                                                        .robotRelativeForcesYNewtons())),
                                        new PPHolonomicDriveController(
                                                        // PID constants for translation
                                                        new PIDConstants(10, 0, 0),
                                                        // PID constants for rotation
                                                        new PIDConstants(7, 0, 0)),
                                        config,
                                        // Assume the path needs to be flipped for Red vs Blue, this is normally the
                                        // case
                                        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                                        this // Subsystem for requirements
                        );
                } catch (Exception ex) {
                        DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                                        ex.getStackTrace());
                }
        }

        public Command snapToAngle(double angle) {

                PARTsUnit currentRobotAngle = new PARTsUnit(getRotation3d().getAngle(), PARTsUnitType.Angle);
                PARTsUnit goalAngle = new PARTsUnit(currentRobotAngle.to(PARTsUnitType.Angle) + angle,
                                PARTsUnitType.Angle);
                thetaController.setGoal(goalAngle.to(PARTsUnitType.Radian));

                // double pidCalc = thetaController.calculate(currentRobotAngle, goalAngle);
                Rotation2d thetaOutput = new Rotation2d(
                                thetaController.calculate(currentRobotAngle.to(PARTsUnitType.Radian),
                                                goalAngle.to(PARTsUnitType.Radian)));

                return this.runOnce(() -> super.setControl(alignRequest
                                .withVelocityX(0)
                                .withVelocityY(0)
                                .withRotationalRate(thetaOutput.getRadians()))).until(() -> (thetaController.atGoal()));

        }

}
