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
import com.pathplanner.lib.path.PathPlannerPath;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Climber;
import frc.robot.generated.TunerConstants;
import frc.robot.util.IPARTsSubsystem;
import frc.robot.util.LimelightHelpers;
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
    private Vision m_Vision;
    private SwerveRequest.RobotCentric alignRequest;

    private ProfiledPIDController thetaController;
    private ProfiledPIDController xRangeController;
    private ProfiledPIDController yRangeController;

    private double MAX_AIM_VELOCITY = 1.5 * Math.PI; // radd/s
    private double MAX_AIM_ACCELERATION = Math.PI / 2; // rad/s^2
    private double MAX_RANGE_VELOCITY = 1.0; // m/s
    private double MAX_RANGE_ACCELERATION = 1.5; // 0.5; // m/2^s

    // Todo - Tune later
    private double THETA_P = 1.8; // Proprotinal
    private double THETA_I = 0.01; // 0.01; //Gradual corretction
    private double THETA_D = 0.05; // 0.05; //Smooth oscilattions

    private double RANGE_P = 2.0;//1.6;// 0.8;
    private double RANGE_I = 0.04;
    private double RANGE_D = 0.1; // ? ~10x P to prevent oscillation(?)

    // Robot poses.
    private Pose3d initialRobotPose3d;
    private Pose3d currentRobotPose3d;

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
        //throw new UnsupportedOperationException("Unimplemented method 'reset'");
    }

    @Override
    public void log() {
        // TODO Auto-generated method stub
        //throw new UnsupportedOperationException("Unimplemented method 'log'");
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

   /*  public Command alignCommand(Pose2d holdDistance, CommandXboxController controller) {
        updatePoseEstimator();

        // Get init. distance from camera.
        Rotation2d estRot = getEstimatedRotation2d();

        if (estRot == null) {
            partsNT.setBoolean("align/vision/MT2 Status", false);
            return new WaitCommand(0);
        } else {
            partsNT.setBoolean("align/vision/MT2 Status", true);
            Rotation3d rotation = new Rotation3d(estRot);

            initialRobotPose3d = m_Vision.convertToKnownSpace(m_Vision.getPose3d(), rotation);

            super.resetPose(initialRobotPose3d.toPose2d());

            partsLogger.logDouble("align/llPoseX", initialRobotPose3d.getX());
            partsLogger.logDouble("align/llPoseY", initialRobotPose3d.getY());
            partsLogger.logDouble("align/llPoseRot", initialRobotPose3d.getRotation().getAngle());

            // Initialize the aim controller.
            thetaController.reset(
                    new PARTsUnit(initialRobotPose3d.getRotation().getAngle(), PARTsUnitType.Angle)
                            .to(PARTsUnitType.Radian));
            thetaController.setGoal(holdDistance.getRotation().getRadians()); // tx=0 is centered.
            thetaController.setTolerance(new PARTsUnit(2, PARTsUnitType.Angle).to(PARTsUnitType.Radian));

            // Initialize the x-range controller.
            xRangeController.reset(initialRobotPose3d.getX());
            xRangeController.setGoal(holdDistance.getX());
            xRangeController.setTolerance(0.1);

            // Initialize the y-range controller.
            yRangeController.reset(initialRobotPose3d.getY()); // Center to target.
            yRangeController.setGoal(holdDistance.getY()); // Center to target.
            yRangeController.setTolerance(0.1);
        }
            

        Command c = this.run(() -> {
            currentRobotPose3d = new Pose3d(super.getState().Pose);

            partsLogger.logDouble("align/rPoseX", currentRobotPose3d.getX());
            partsLogger.logDouble("align/rPoseY", currentRobotPose3d.getY());
            partsLogger.logDouble("align/rPoseRot", currentRobotPose3d.getRotation().getAngle());

            partsNT.setDouble("align/rPoseX", currentRobotPose3d.getX());
            partsNT.setDouble("align/rPoseY", currentRobotPose3d.getY());
            partsNT.setDouble("align/rPoseRot", currentRobotPose3d.getRotation().getAngle());

            Rotation2d thetaOutput = new Rotation2d(
                    thetaController.calculate(currentRobotPose3d.getRotation().toRotation2d().getRadians()));

            Pose2d rangeOutput = new Pose2d(
                    xRangeController.calculate(currentRobotPose3d.getX(), holdDistance.getX()),
                    yRangeController.calculate(currentRobotPose3d.getY(), holdDistance.getY()),
                    null);

            // Get dist. from drivetrain.

            Translation2d translation = new Translation2d(rangeOutput.getX(), rangeOutput.getY());

            partsLogger.logDouble("align/Output/thetaController", thetaOutput.getDegrees());
            partsLogger.logDouble("align/Output/rangeControllerX", rangeOutput.getX());
            partsLogger.logDouble("align/Output/rangeControllerY", rangeOutput.getY());

            partsNT.setDouble("align/Output/thetaController", thetaOutput.getDegrees());
            partsNT.setDouble("align/Output/rangeControllerX", rangeOutput.getX());
            partsNT.setDouble("align/Output/rangeControllerY", rangeOutput.getY());

            partsLogger.logBoolean("align/Goal/thetaController", thetaController.atGoal());
            partsLogger.logBoolean("align/Goal/rangeControllerX", xRangeController.atGoal());
            partsLogger.logBoolean("align/Goal/rangeControllerYGoal", yRangeController.atGoal());

            partsNT.setBoolean("align/Goal/thetaController", thetaController.atGoal());
            partsNT.setBoolean("align/Goal/rangeControllerX", xRangeController.atGoal());
            partsNT.setBoolean("align/Goal/rangeControllerY", yRangeController.atGoal());

            updatePoseEstimator();
            super.setControl(alignRequest
                    .withVelocityX(translation.getX())
                    .withVelocityY(translation.getY())
                    .withRotationalRate(thetaOutput.getRadians()));
        })
                .onlyIf(() -> m_Vision.isTarget())
                .until(() -> (xRangeController.atGoal() &&
                        yRangeController.atGoal() &&
                        thetaController.atGoal())
                        || (controller != null &&
                                (Math.abs(controller.getLeftX()) > 0.1 ||
                                        Math.abs(controller.getLeftY()) > 0.1 ||
                                        Math.abs(controller.getRightX()) > 0.1)))
                .andThen(this.runOnce(() -> super.setControl(alignRequest
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0))));

        c.setName("align");
        return c;
    }
        */

    /*---------------------------------- Custom Private Functions ---------------------------------*/
    private void initialize() {
        initializeClasses();
        initializePoseEstimator();
        initializeControllers();
        sendToDashboard();
        configureAutoBuilder();
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

        //partsNT.putSmartDashboardSendable("Align", alignCommand(new Pose2d(-1, 0, new Rotation2d()), null));
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

    private void initializeControllers() {
        alignRequest = new SwerveRequest.RobotCentric()
                .withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1).withRotationalDeadband(0.1);

        thetaController = new ProfiledPIDController(THETA_P, THETA_I, THETA_D,
                new TrapezoidProfile.Constraints(MAX_AIM_VELOCITY, MAX_AIM_ACCELERATION));
        thetaController.enableContinuousInput(-Math.PI, Math.PI); // Wrpa from -pi to ip

        xRangeController = new ProfiledPIDController(RANGE_P, RANGE_I, RANGE_D,
                new TrapezoidProfile.Constraints(MAX_RANGE_VELOCITY, MAX_RANGE_ACCELERATION));
        yRangeController = new ProfiledPIDController(RANGE_P, RANGE_I, RANGE_D,
                new TrapezoidProfile.Constraints(MAX_RANGE_VELOCITY, MAX_RANGE_ACCELERATION));

    }

    private void initializeClasses() {
        partsNT = new PARTsNT(this);
        partsLogger = new PARTsLogger(this);
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
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }
    
    public Command snapToAngle(double angle) {
    
        PARTsUnit currentRobotAngle = new PARTsUnit(getRotation3d().getAngle(), PARTsUnitType.Angle);
        PARTsUnit goalAngle = new PARTsUnit(currentRobotAngle.to(PARTsUnitType.Angle) + angle, PARTsUnitType.Angle);
        thetaController.setGoal(goalAngle.to(PARTsUnitType.Radian));

       //double pidCalc = thetaController.calculate(currentRobotAngle, goalAngle);
        Rotation2d thetaOutput = new Rotation2d(
                    thetaController.calculate(currentRobotAngle.to(PARTsUnitType.Radian), goalAngle.to(PARTsUnitType.Radian)));

        return this.runOnce(() -> super.setControl(alignRequest
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(thetaOutput.getRadians()))).until(() -> (thetaController.atGoal()));
        
          
    }

}
