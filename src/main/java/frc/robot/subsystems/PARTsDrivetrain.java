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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
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
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.util.AprilTagData;
import frc.robot.util.IPARTsSubsystem;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.PARTsCommandController;
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
        private Pose3d initialPose3d;
        private Pose3d currentRobotPose3d;
        private Pose3d currentVisionPose3d;
        private double initialRobotAngleRad;

        Pose2d initialPose2d;
        double turnPosNeg;
        double skewVal;

        double tagID;

        public PARTsDrivetrain(Vision vision,
                        SwerveDrivetrainConstants drivetrainConstants,
                        SwerveModuleConstants<?, ?, ?>... modules) {
                super(drivetrainConstants, modules);
                this.m_vision = vision;

                initialize();
        }

        public PARTsDrivetrain(Vision vision,
                        SwerveDrivetrainConstants drivetrainConstants,
                        double odometryUpdateFrequency,
                        SwerveModuleConstants<?, ?, ?>... modules) {
                super(drivetrainConstants, odometryUpdateFrequency, modules);
                this.m_vision = vision;

                initialize();
        }

        public PARTsDrivetrain(Vision vision,
                        SwerveDrivetrainConstants drivetrainConstants,
                        double odometryUpdateFrequency,
                        Matrix<N3, N1> odometryStandardDeviation,
                        Matrix<N3, N1> visionStandardDeviation,
                        SwerveModuleConstants<?, ?, ?>... modules) {
                super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                                modules);
                this.m_vision = vision;

                initialize();

        }

        /*-------------------------------- Generic Subsystem Functions --------------------------------*/
        @Override
        public void outputTelemetry() {
                partsNT.setDouble("Vision/Vision x", currentVisionPose3d.getX());
                partsNT.setDouble("Vision/Vision y", currentVisionPose3d.getY());
                partsNT.setDouble("Vision/Vision z",
                                new PARTsUnit(currentVisionPose3d.getRotation().getAngle(), PARTsUnitType.Radian)
                                                .to(PARTsUnitType.Angle));

                partsNT.setBoolean("align/validTag", tagID > 0);
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

                currentVisionPose3d = m_vision.getPose3d();
                try {
                        tagID = m_vision.getTargetID();
                } catch (ArrayIndexOutOfBoundsException e) {
                        tagID = -1;
                }
                updatePoseEstimator();
        }

        /*---------------------------------- Custom Public Functions ----------------------------------*/

        public Command alignCommand(Pose2d holdDistance, PARTsCommandController controller) {
                Command c = new FunctionalCommand(
                                () -> {

                                        double[] botPoseTargetSpace = LimelightHelpers.getLimelightNTDoubleArray("",
                                                        "botpose_targetspace");

                                        initialPose3d = m_vision.convertToKnownSpace(currentVisionPose3d);

                                        turnPosNeg = -Math.signum(botPoseTargetSpace[4]);

                                        initialPose2d = new Pose2d(initialPose3d.getX(), initialPose3d.getY(),
                                                        new Rotation2d(initialPose3d.getRotation().getAngle()
                                                                        * turnPosNeg));

                                        // Feel like it needs to init before each estimation. remove if does not work
                                        initializePoseEstimator();
                                        setPoseEstimatorVisionMeasurement(initialPose2d,
                                                        System.currentTimeMillis() / 1000);

                                        // Initialize the aim controller.
                                        thetaController.reset(
                                                        new PARTsUnit(initialPose3d.getRotation()
                                                                        .getAngle(),
                                                                        PARTsUnitType.Angle)
                                                                        .to(PARTsUnitType.Radian));
                                        thetaController.setGoal(holdDistance.getRotation().getRadians()); // tx=0
                                                                                                          // is
                                                                                                          // centered.
                                        thetaController.setTolerance(
                                                        Constants.Drivetrain.thetaControllerTolerance
                                                                        .to(PARTsUnitType.Radian));

                                        // Initialize the x-range controller.
                                        xRangeController.reset(initialPose3d.getX());
                                        xRangeController.setGoal(holdDistance.getX());
                                        xRangeController.setTolerance(Constants.Drivetrain.xRControllerTolerance
                                                        .to(PARTsUnitType.Meter));

                                        // Initialize the y-range controller.
                                        yRangeController.reset(initialPose3d.getY()); // Center to target.
                                        yRangeController.setGoal(holdDistance.getY()); // Center to target.
                                        yRangeController.setTolerance(Constants.Drivetrain.yRControllerTolerance
                                                        .to(PARTsUnitType.Meter));

                                        alignCommandInitTelemetry();
                                },
                                () -> {
                                        currentRobotPose3d = new Pose3d(m_poseEstimator.getEstimatedPosition());

                                        Rotation2d thetaOutput = new Rotation2d(
                                                        thetaController.calculate(currentRobotPose3d.getRotation()
                                                                        .toRotation2d().getRadians()));

                                        Pose2d rangeOutput = new Pose2d(
                                                        xRangeController.calculate(currentRobotPose3d.getX(),
                                                                        holdDistance.getX()),
                                                        yRangeController.calculate(currentRobotPose3d.getY(),
                                                                        holdDistance.getY()),
                                                        null);

                                        // Get dist. from drivetrain.

                                        Translation2d translation = new Translation2d(rangeOutput.getX(),
                                                        rangeOutput.getY());

                                        super.setControl(new SwerveRequest.ApplyRobotSpeeds()
                                                        .withSpeeds(new ChassisSpeeds(translation.getX(),
                                                                        translation.getY(), thetaOutput.getRadians())));

                                        alignCommandExecuteTelemetry(thetaOutput, rangeOutput);
                                },
                                (Boolean b) -> {
                                        super.setControl(alignRequest
                                                        .withVelocityX(0)
                                                        .withVelocityY(0)
                                                        .withRotationalRate(0));
                                },
                                () -> (tagID <= 0 || (xRangeController.atGoal() &&
                                                yRangeController.atGoal() &&
                                                thetaController.atGoal())
                                                || (controller != null &&
                                                                (Math.abs(controller.getLeftX()) > 0.1 ||
                                                                                Math.abs(controller
                                                                                                .getLeftY()) > 0.1
                                                                                ||
                                                                                Math.abs(controller
                                                                                                .getRightX()) > 0.1))),
                                this);
                c.setName("align");
                return c;
        }

        public Command alignDebugCommand() {
                Command c = new FunctionalCommand(
                                () -> {
                                        initialPose3d = m_vision.convertToKnownSpace(currentVisionPose3d);

                                        initialPose3d = initialPose3d.plus(new Transform3d(0,
                                                        new PARTsUnit(Constants.Drivetrain.leftSideOffset,
                                                                        PARTsUnitType.Inch)
                                                                        .to(PARTsUnitType.Meter),
                                                        0, new Rotation3d()));

                                        super.resetPose(initialPose3d.toPose2d());

                                        alignCommandInitTelemetry();
                                },
                                () -> {

                                },
                                (Boolean b) -> {
                                        super.setControl(alignRequest
                                                        .withVelocityX(0)
                                                        .withVelocityY(0)
                                                        .withRotationalRate(0));

                                },
                                () -> (true),
                                this);
                c.setName("alignDebug");
                return c;
        }

        /*---------------------------------- Custom Private Functions ---------------------------------*/
        private void alignCommandInitTelemetry() {
                partsNT.setDouble("align/startMS", System.currentTimeMillis());

                partsLogger.logDouble("align/llPoseX", initialPose3d.getX());
                partsLogger.logDouble("align/llPoseY", initialPose3d.getY());
                partsLogger.logDouble("align/llPoseRot",
                                new PARTsUnit(initialPose3d.getRotation()
                                                .getAngle(), PARTsUnitType.Radian)
                                                .to(PARTsUnitType.Angle));

                partsNT.setDouble("align/llPoseX", initialPose3d.getX());
                partsNT.setDouble("align/llPoseY", initialPose3d.getY());
                partsNT.setDouble("align/llPoseRot",
                                new PARTsUnit(initialPose3d.getRotation()
                                                .getAngle(), PARTsUnitType.Radian)
                                                .to(PARTsUnitType.Angle));

                partsLogger.logDouble("align/thetaControllerSetpoint",
                                thetaController.getSetpoint().position);
                partsNT.setDouble("align/thetaControllerSetpoint",
                                thetaController.getSetpoint().position);

                partsNT.setDouble("align/testPoseX", initialPose2d.getX());
                partsNT.setDouble("align/testPoseY", initialPose2d.getY());
                partsNT.setDouble("align/testPoseRot", new PARTsUnit(initialPose2d
                                .getRotation().getRadians(), PARTsUnitType.Radian)
                                .to(PARTsUnitType.Angle));

                partsNT.setDouble("align/turnPosNeg", turnPosNeg);
                partsNT.setDouble("align/aprilTagID", tagID);
        }

        private void alignCommandExecuteTelemetry(Rotation2d thetaOutput, Pose2d rangeOutput) {
                partsLogger.logDouble("align/rPoseX",
                                new PARTsUnit(currentRobotPose3d.getX(), PARTsUnitType.Meter)
                                                .to(PARTsUnitType.Inch));
                partsLogger.logDouble("align/rPoseY",
                                new PARTsUnit(currentRobotPose3d.getY(), PARTsUnitType.Meter)
                                                .to(PARTsUnitType.Inch));
                partsLogger.logDouble("align/rPoseRot",
                                new PARTsUnit(currentRobotPose3d.getRotation().getAngle(),
                                                PARTsUnitType.Radian).to(PARTsUnitType.Angle));

                partsNT.setDouble("align/rPoseX",
                                new PARTsUnit(currentRobotPose3d.getX(), PARTsUnitType.Meter)
                                                .to(PARTsUnitType.Inch));
                partsNT.setDouble("align/rPoseY",
                                new PARTsUnit(currentRobotPose3d.getY(), PARTsUnitType.Meter)
                                                .to(PARTsUnitType.Inch));
                partsNT.setDouble("align/rPoseRot",
                                new PARTsUnit(currentRobotPose3d.getRotation().getAngle(),
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
                initializeClasses();
                initializeControllers();
                sendToDashboard();
                configureAutoBuilder();
                AprilTagData.InitAprilTagObjects();
                frontLeftModule = getModule(0);
                frontRightModule = getModule(1);
                backLeftModule = getModule(2);
                backRightModule = getModule(3);
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

                partsNT.putSmartDashboardSendable("Align", alignCommand(new Pose2d(-1, 0, new Rotation2d()), null));
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

        private void setPoseEstimatorVisionMeasurement(Pose2d measurements, double timestampSeconds) {
                /*if (Math.abs(super.getPigeon2().getRate()) > 720) // if our angular velocity is greater than 720
                                                                  // degrees per
                                                                  // second, ignore vision updates
                {
                        doRejectUpdate = true;
                }

                if (!doRejectUpdate) {

                }*/

                m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                m_poseEstimator.addVisionMeasurement(
                                measurements,
                                timestampSeconds);
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
