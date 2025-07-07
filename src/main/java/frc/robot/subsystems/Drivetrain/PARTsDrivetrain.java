package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.IOException;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.Telemetry;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.generated.TunerConstants;
import frc.robot.util.Field.Field;
import frc.robot.util.PARTs.PARTsCommandController;
import frc.robot.util.PARTs.PARTsCommandUtils;
import frc.robot.util.PARTs.PARTsLogger;
import frc.robot.util.PARTs.PARTsNT;
import frc.robot.util.PARTs.PARTsUnit;
import frc.robot.util.PARTs.PARTsUnit.PARTsUnitType;
import frc.robot.util.PARTs.Interfaces.IPARTsSubsystem;

public class PARTsDrivetrain extends CommandSwerveDrivetrain implements IPARTsSubsystem {
        /*-------------------------------- Private instance variables ---------------------------------*/
        private boolean fineGrainDrive = false;
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

        private Telemetry telemetryLogger;
        /// check which are needed.
        private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

        private SwerveModule<TalonFX, TalonFX, CANcoder> frontRightModule;
        private SwerveModule<TalonFX, TalonFX, CANcoder> frontLeftModule;
        private SwerveModule<TalonFX, TalonFX, CANcoder> backRightModule;
        private SwerveModule<TalonFX, TalonFX, CANcoder> backLeftModule;

        private FieldObject2d fieldObject2d;
        private FieldObject2d targetObject2d;

        private PARTsNT partsNT;
        private PARTsLogger partsLogger;

        private Timer alignTimer;
        private PARTsUnit drivetrainVelocityX;
        private PARTsUnit drivetrainVelocityY;
        private boolean timerElapsed = false;

        // Vision Variables
        private SwerveRequest.FieldCentric alignRequest;

        private ProfiledPIDController thetaController;
        private ProfiledPIDController xRangeController;
        private ProfiledPIDController yRangeController;
        private PathPlannerPath path;
        private Supplier<PathPlannerPath> pathSupplier = () -> path;

        public PARTsDrivetrain(
                        SwerveDrivetrainConstants DrivetrainConstants,
                        SwerveModuleConstants<?, ?, ?>... modules) {
                super(DrivetrainConstants, modules);

                instantiate();
        }

        public PARTsDrivetrain(
                        SwerveDrivetrainConstants DrivetrainConstants,
                        double odometryUpdateFrequency,
                        SwerveModuleConstants<?, ?, ?>... modules) {
                super(DrivetrainConstants, odometryUpdateFrequency, modules);

                instantiate();
        }

        public PARTsDrivetrain(
                        SwerveDrivetrainConstants DrivetrainConstants,
                        double odometryUpdateFrequency,
                        Matrix<N3, N1> odometryStandardDeviation,
                        Matrix<N3, N1> visionStandardDeviation,
                        SwerveModuleConstants<?, ?, ?>... modules) {
                super(DrivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                                modules);

                instantiate();

        }

        private void instantiate() {
                frontLeftModule = getModule(0);
                frontRightModule = getModule(1);
                backLeftModule = getModule(2);
                backRightModule = getModule(3);
                initializeClasses();
                initializeControllers();
                sendToDashboard();
                configureAutoBuilder();
                fieldObject2d = Field.FIELD2D.getObject("Robot");
                targetObject2d = Field.FIELD2D.getObject("target pose");
                telemetryLogger = new Telemetry(MaxSpeed);
                registerTelemetry(telemetryLogger::telemeterize);
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
                fieldObject2d.setPose(getFieldCentricPose());
        }

        /*---------------------------------- Custom Public Functions ----------------------------------*/

        public SwerveRequest.FieldCentric getFieldCentricDriveRequest() {
                /* Setting up bindings for necessary control of the swerve drive platform */
                return new SwerveRequest.FieldCentric()
                                .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                                .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
        }

        public SwerveRequest.SwerveDriveBrake getBrakeDriveRequest() {
                return new SwerveRequest.SwerveDriveBrake();
        }

        public SwerveRequest.PointWheelsAt getPointDriveRequest() {
                return new SwerveRequest.PointWheelsAt();
        }

        public void toggleFineGrainDrive() {
                fineGrainDrive = !fineGrainDrive;
        }

        public Command commandDefault(PARTsCommandController controller) {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                return PARTsCommandUtils.setCommandName("commandDefault", applyRequest(() -> {
                        double limit = MaxSpeed;
                        if (fineGrainDrive)
                                limit *= 0.25;
                        return getFieldCentricDriveRequest().withVelocityX(-controller.getLeftY() * limit) // Drive forward with negative Y
                                        // (forward)
                                        .withVelocityY(-controller.getLeftX() * limit) // Drive left with negative
                                        // X (left)
                                        .withRotationalRate(-controller.getRightX() * MaxAngularRate); // Drive
                                                                                                       // counterclockwise
                                                                                                       // with
                                                                                                       // negative
                                                                                                       // X (left)
                }));
        }

        public Command commandBrake() {
                return PARTsCommandUtils.setCommandName("commandBrake",
                                applyRequest(() -> getBrakeDriveRequest()));
        }

        public Command commandPointWheels(PARTsCommandController controller) {
                return PARTsCommandUtils.setCommandName("commandPointWheels", applyRequest(() -> getPointDriveRequest()
                                .withModuleDirection(new Rotation2d(-controller.getLeftY(),
                                                -controller.getLeftX()))));
        }

        public Command commandSeedFieldCentric() {
                return PARTsCommandUtils.setCommandName("commandSeedFieldCentric",
                                this.runOnce(() -> super.seedFieldCentric()));
        }

        public Command commandAlign(Pose2d goalPose) {
                return commandAlign(() -> goalPose);
        }

        public Command commandAlign(Supplier<Pose2d> goalPose) {
                Command c = new FunctionalCommand(
                                () -> {
                                        timerElapsed = false;
                                        targetObject2d.setPose(goalPose.get());
                                        alignTimer = new Timer();
                                        alignTimer.start();

                                        // Initialize the aim controller.
                                        thetaController.reset(getFieldCentricPose().getRotation()
                                                        .getRadians());

                                        thetaController.setGoal(goalPose.get().getRotation().getRadians()); // tx=0
                                                                                                            // is
                                                                                                            // centered.
                                        thetaController.setTolerance(
                                                        DrivetrainConstants.thetaControllerTolerance
                                                                        .to(PARTsUnitType.Radian));

                                        // Initialize the x-range controller.
                                        xRangeController.reset(getFieldCentricPose().getX());
                                        xRangeController.setGoal(goalPose.get().getX());
                                        xRangeController.setTolerance(DrivetrainConstants.xRControllerTolerance
                                                        .to(PARTsUnitType.Meter));

                                        // Initialize the y-range controller.
                                        yRangeController.reset(getFieldCentricPose().getY()); // Center
                                        // to
                                        // target.
                                        yRangeController.setGoal(goalPose.get().getY()); // Center to target.
                                        yRangeController.setTolerance(DrivetrainConstants.yRControllerTolerance
                                                        .to(PARTsUnitType.Meter));

                                        alignCommandInitTelemetry(goalPose.get());
                                },
                                () -> {
                                        Pose2d pose = getFieldCentricPose();
                                        Transform2d diff = goalPose.get().minus(pose);

                                        drivetrainVelocityX = getXVelocity();
                                        drivetrainVelocityY = getYVelocity();

                                        if (Math.max(drivetrainVelocityX.getMagnitude(),
                                                        drivetrainVelocityY.getMagnitude()) > 0.01
                                                        || Math.abs(diff.getTranslation()
                                                                        .getNorm()) > PARTsUnit.InchesToMeters
                                                                                        .apply(24.0)) {
                                                alignTimer.reset();
                                        }

                                        if (alignTimer.hasElapsed(0.25)) {
                                                timerElapsed = true;
                                        }

                                        Rotation2d thetaOutput = new Rotation2d(
                                                        thetaController.calculate(
                                                                        getFieldCentricPose().getRotation()
                                                                                        .getRadians()));

                                        Pose2d rangeOutput = new Pose2d(
                                                        xRangeController.calculate(getFieldCentricPose().getX(),
                                                                        goalPose.get().getX()),
                                                        yRangeController.calculate(getFieldCentricPose().getY(),
                                                                        goalPose.get().getY()),
                                                        null);

                                        // Get dist. from drivetrain.

                                        Translation2d translation = new Translation2d(rangeOutput.getX(),
                                                        rangeOutput.getY());

                                        super.setControl(alignRequest
                                                        .withVelocityX(translation.getX())
                                                        .withVelocityY(translation.getY())
                                                        .withRotationalRate(thetaOutput.getRadians()));

                                        alignCommandExecuteTelemetry(thetaOutput, rangeOutput, diff);
                                },
                                (Boolean b) -> {
                                        super.setControl(alignRequest
                                                        .withVelocityX(0)
                                                        .withVelocityY(0)
                                                        .withRotationalRate(0));
                                        timerElapsed = false;
                                        alignTimer.reset();
                                },
                                () -> ((xRangeController.atGoal() &&
                                                yRangeController.atGoal() &&
                                                thetaController.atGoal()) || timerElapsed));
                c.setName("align");
                return c;
        }

        public Pose2d getPose() {
                return super.getState().Pose;
        }

        public Pose2d getFieldCentricPose() {
                return Field.conditionallyTransformToOppositeAlliance(getPose());
        }

        public Command commandSnapToAngle(double angle) {

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

        public void setChassisSpeeds(ChassisSpeeds robotSpeeds) {
                setControl(new SwerveRequest.RobotCentric().withVelocityX(robotSpeeds.vxMetersPerSecond)
                                .withVelocityY(robotSpeeds.vyMetersPerSecond)
                                .withRotationalRate(robotSpeeds.omegaRadiansPerSecond));
        }

        public PARTsUnit getXVelocity() {
                return new PARTsUnit(super.getState().Speeds.vxMetersPerSecond, PARTsUnitType.MetersPerSecond);
        }

        public PARTsUnit getYVelocity() {
                return new PARTsUnit(super.getState().Speeds.vyMetersPerSecond, PARTsUnitType.MetersPerSecond);
        }

        public Command commandPathFindToPath(String pathname) {
                try {
                        // Load the path we want to pathfind to and follow
                        PathPlannerPath path = PathPlannerPath.fromPathFile(pathname);

                        // Create the constraints to use while pathfinding. The constraints defined in
                        // the path will only be used for the path.
                        PathConstraints constraints = new PathConstraints(
                                        0.5, 0.5,
                                        PARTsUnit.DegreesToRadians.apply(540.0),
                                        PARTsUnit.DegreesToRadians.apply(720.0));

                        // Since AutoBuilder is configured, we can use it to build pathfinding commands
                        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                                        path,
                                        constraints);
                        pathfindingCommand.setName("pathFindToPathCommand");
                        System.out.println("anything");
                        return pathfindingCommand;

                } catch (IOException e) {
                        e.printStackTrace();
                } catch (FileVersionException e) {
                        // TODO Auto-generated catch block
                        e.printStackTrace();
                } catch (ParseException e) {
                        // TODO Auto-generated catch block
                        e.printStackTrace();
                }
                return new WaitCommand(0);
        }

        public Command commandPathFindToPose(Pose2d pose) {

                // Create the constraints to use while pathfinding. The constraints defined in
                // the path will only be used for the path.
                PathConstraints constraints = new PathConstraints(
                                1, 1,
                                PARTsUnit.DegreesToRadians.apply(540.0),
                                PARTsUnit.DegreesToRadians.apply(720.0));

                // Since AutoBuilder is configured, we can use it to build pathfinding commands
                Command pathfindingCommand = AutoBuilder.pathfindToPose(
                                Field.conditionallyTransformToOppositeAlliance(pose),
                                constraints, 0.0); // Goal end velocity in meters/sec
                pathfindingCommand.setName("pathFindToPoseCommand");
                
                return pathfindingCommand;
        }

        /*public Command commandPathOnTheFly(Pose2d pose) {
                
                Command c = this.runOnce(()->{
                PathConstraints constraints = new PathConstraints(0.01, 0.01, 2 * Math.PI, 4 * Math.PI); // The
                                                                                                       // constraints
                                                                                                       // for this path.
                // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); //
                // You can also use unlimited constraints, only limited by motor torque and
                // nominal battery voltage
                Pose2d middlePoint = pose.transformBy(new Transform2d(1 + RobotConstants.frontRobotVisionOffset.getValue(),0, new Rotation2d()));
                Pose2d lastPoint = pose.transformBy(new Transform2d(RobotConstants.frontRobotVisionOffset.getValue(),0, new Rotation2d()));
                // Create the path using the waypoints created above
                path = new PathPlannerPath(
                                PathPlannerPath.waypointsFromPoses(getPose(),middlePoint,lastPoint),
                                constraints,
                                null, // The ideal starting state, this is only relevant for pre-planned paths, so can
                                      // be null for on-the-fly paths.
                                new GoalEndState(0.0, pose.getRotation()) // Goal end state. You can set a
                                                                                   // holonomic rotation here. If using
                                                                                   // a differential drivetrain, the
                                                                                   // rotation will have no effect.
                );
        
                // Prevent the path from being flipped if the coordinates are already correct
                path.preventFlipping = false;
        }).andThen(AutoBuilder.followPath(pathSupplier.get()));
                c.setName("commandPathOnTheFly");
                return c;
        }
        */

        /*---------------------------------- Custom Private Functions ---------------------------------*/
        private void alignCommandInitTelemetry(Pose2d holdDist) {
                partsNT.putDouble("align/holdDistX", new PARTsUnit(holdDist.getX(), PARTsUnitType.Meter)
                                .to(PARTsUnitType.Inch));
                partsNT.putDouble("align/holdDistY", new PARTsUnit(holdDist.getY(), PARTsUnitType.Meter)
                                .to(PARTsUnitType.Inch));
                partsNT.putDouble("align/holdDistRot",
                                new PARTsUnit(holdDist.getRotation().getRadians(), PARTsUnitType.Radian)
                                                .to(PARTsUnitType.Angle));

                partsLogger.logDouble("align/thetaControllerSetpoint",
                                thetaController.getSetpoint().position);
                partsNT.putDouble("align/thetaControllerSetpoint",
                                thetaController.getSetpoint().position);

        }

        private void alignCommandExecuteTelemetry(Rotation2d thetaOutput, Pose2d rangeOutput, Transform2d diff) {
                partsLogger.logDouble("align/rPoseX",
                                new PARTsUnit(getPose().getX(), PARTsUnitType.Meter)
                                                .to(PARTsUnitType.Inch));
                partsLogger.logDouble("align/rPoseY",
                                new PARTsUnit(getPose().getY(), PARTsUnitType.Meter)
                                                .to(PARTsUnitType.Inch));
                partsLogger.logDouble("align/rPoseRot",
                                new PARTsUnit(getPose().getRotation().getRadians(),
                                                PARTsUnitType.Radian).to(PARTsUnitType.Angle));

                partsNT.putDouble("align/rPoseX",
                                new PARTsUnit(getPose().getX(), PARTsUnitType.Meter)
                                                .to(PARTsUnitType.Inch));
                partsNT.putDouble("align/rPoseY",
                                new PARTsUnit(getPose().getY(), PARTsUnitType.Meter)
                                                .to(PARTsUnitType.Inch));
                partsNT.putDouble("align/rPoseRot",
                                new PARTsUnit(getPose().getRotation().getRadians(),
                                                PARTsUnitType.Radian).to(PARTsUnitType.Angle));

                partsLogger.logDouble("align/Output/thetaController", thetaOutput.getDegrees());
                partsLogger.logDouble("align/Output/rangeControllerX", rangeOutput.getX());
                partsLogger.logDouble("align/Output/rangeControllerY", rangeOutput.getY());

                partsNT.putDouble("align/Output/thetaController", thetaOutput.getDegrees());
                partsNT.putDouble("align/Output/rangeControllerX", rangeOutput.getX());
                partsNT.putDouble("align/Output/rangeControllerY", rangeOutput.getY());

                partsLogger.logBoolean("align/Goal/thetaAtGoal", thetaController.atGoal());
                partsLogger.logBoolean("align/Goal/rangeXAtGoal",
                                xRangeController.atGoal());
                partsLogger.logBoolean("align/Goal/rangeYAtGoal",
                                yRangeController.atGoal());

                partsNT.putDouble("align/Goal/x setpoint", xRangeController.getSetpoint().position);
                partsNT.putDouble("align/Goal/y setpoint", yRangeController.getSetpoint().position);
                partsNT.putDouble("align/Goal/x setpoint", thetaController.getSetpoint().position);

                partsNT.putBoolean("align/Goal/thetaAtGoal", thetaController.atGoal());
                partsNT.putBoolean("align/Goal/rangeXAtGoal", xRangeController.atGoal());
                partsNT.putBoolean("align/Goal/rangeYAtGoal", yRangeController.atGoal());

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

                partsNT.putDouble("align/Output/PosErrorX",
                                xRangeController.getPositionError());
                partsNT.putDouble("align/Output/PosErrorY",
                                yRangeController.getPositionError());
                partsNT.putDouble("align/Output/thetaPosError",
                                thetaController.getPositionError());

                partsNT.putDouble("align/Output/velocityErrorX",
                                xRangeController.getVelocityError());
                partsNT.putDouble("align/Output/velocityErrorY",
                                yRangeController.getVelocityError());
                partsNT.putDouble("align/Output/thetaVelocityError",
                                thetaController.getVelocityError());

                partsNT.putDouble("align/timer", alignTimer.get());
                partsNT.putBoolean("align/timerHasElapsed", timerElapsed);

                partsNT.putDouble("align/pigeonMovementX", drivetrainVelocityX.getValue());
                partsNT.putDouble("align/pigeonMovementY", drivetrainVelocityY.getValue());
                partsNT.putDouble("align/goalPoseError", Math.abs(diff.getTranslation().getNorm()));
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

        private void initializeControllers() {
                alignRequest = new SwerveRequest.FieldCentric()
                                .withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1)
                                .withRotationalDeadband(0.1);

                thetaController = new ProfiledPIDController(DrivetrainConstants.THETA_P, DrivetrainConstants.THETA_I,
                                DrivetrainConstants.THETA_D,
                                new TrapezoidProfile.Constraints(DrivetrainConstants.MAX_AIM_VELOCITY,
                                                DrivetrainConstants.MAX_AIM_ACCELERATION));
                thetaController.enableContinuousInput(-Math.PI, Math.PI); // Wrpa from -pi to ip

                xRangeController = new ProfiledPIDController(DrivetrainConstants.RANGE_X_P,
                                DrivetrainConstants.RANGE_I,
                                DrivetrainConstants.RANGE_D,
                                new TrapezoidProfile.Constraints(DrivetrainConstants.MAX_RANGE_VELOCITY,
                                                DrivetrainConstants.MAX_RANGE_ACCELERATION));
                yRangeController = new ProfiledPIDController(DrivetrainConstants.RANGE_Y_P,
                                DrivetrainConstants.RANGE_I,
                                DrivetrainConstants.RANGE_D,
                                new TrapezoidProfile.Constraints(DrivetrainConstants.MAX_RANGE_VELOCITY,
                                                DrivetrainConstants.MAX_RANGE_ACCELERATION));

        }

        private void initializeClasses() {
                partsNT = new PARTsNT(this);
                partsLogger = new PARTsLogger(this);
        }

        /*---------------------------------- Override Functions ----------------------------------*/
        @Override
        public void addVisionMeasurement(Pose2d measurement, double timestamp) {
                super.addVisionMeasurement(measurement, Utils.fpgaToCurrentTime(timestamp));
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
                                        this::getPose, // Supplier of current robot pose
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
                                        () -> false,
                                        this // Subsystem for requirements
                        );
                } catch (Exception ex) {
                        DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder",
                                        ex.getStackTrace());
                }
        }
}
