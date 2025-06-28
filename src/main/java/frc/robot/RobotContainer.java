package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.cmds.coral.L4ScoreCoral;
import frc.robot.subsystems.Candle;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Candle.CandleState;
import frc.robot.subsystems.PARTsDrivetrain;
import frc.robot.util.IPARTsSubsystem;
import frc.robot.util.PARTsButtonBoxController;
import frc.robot.util.PARTsCommandController;
import frc.robot.util.PARTsDashboard;
import frc.robot.util.PARTsNT;
import frc.robot.util.PARTsUnit;
import frc.robot.util.PARTsController.ControllerType;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimelightVision;

public class RobotContainer {
        private boolean fineGrainDrive = false;
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry telemetryLogger = new Telemetry(MaxSpeed);

        private final PARTsCommandController driveController = new PARTsCommandController(0, ControllerType.DS5);
        private final PARTsCommandController operatorController = new PARTsCommandController(1, ControllerType.XBOX);
        private final PARTsButtonBoxController buttonBoxController = new PARTsButtonBoxController(2);

        private boolean elevatorManualControl = false;

        /* Subsystems */

        public final Candle candle = new Candle();

        private final Elevator elevator = new Elevator(candle);
        // private final ElevatorSysId elevator = new ElevatorSysId();

        private final Coral coral = new Coral(candle, elevator);

        public final PARTsDrivetrain drivetrain = new PARTsDrivetrain(
                        TunerConstants.DrivetrainConstants,
                        TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft,
                        TunerConstants.BackRight);

        private final LimelightVision vision = new LimelightVision(drivetrain);

        private final ArrayList<IPARTsSubsystem> subsystems = new ArrayList<IPARTsSubsystem>(
                        Arrays.asList(candle, coral, elevator, drivetrain, vision));

        private SendableChooser<Command> autoChooser;

        private PARTsNT partsNT = new PARTsNT("RobotContainer");

        /** End Subsystems */

        BooleanSupplier manualElevatorControlSupplier = () -> elevatorManualControl;
        BooleanSupplier escapeBooleanSupplier;

        public RobotContainer() {
                escapeBooleanSupplier = buttonBoxController.negative3Trigger()
                                .or(driveController.axisMagnitudeGreaterThan(0, 0.5))
                                .or(driveController.axisMagnitudeGreaterThan(1, 0.5))
                                .or(driveController.rightTrigger());

                // configureAutonomousCommands();
                configureBindings();
                partsNT.putSmartDashboardSendable("field", Field.FIELD2D);
                vision.resetPose();
        }

        private void configureBindings() {
                // * */
                // =============================================================================================
                // * */ ------------------------------------- DriveTrain
                // * */ -------------------------------------------
                // * */
                // ---------------------------------------------------------------------------------------------

                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                Command driveCommand = drivetrain.applyRequest(() -> {
                        double limit = MaxSpeed;
                        if (fineGrainDrive)
                                limit *= 0.25;
                        return drive.withVelocityX(-driveController.getLeftY() * limit) // Drive forward with negative Y
                                        // (forward)
                                        .withVelocityY(-driveController.getLeftX() * limit) // Drive left with negative
                                        // X (left)
                                        .withRotationalRate(-driveController.getRightX() * MaxAngularRate); // Drive
                                                                                                            // counterclockwise
                                                                                                            // with
                                                                                                            // negative
                                                                                                            // X (left)
                });
                driveCommand.setName("DefaultDrive");
                // Drivetrain will execute this command periodically
                drivetrain.setDefaultCommand(driveCommand);

                // fine grain controls
                driveController.rightBumper().onTrue(Commands.runOnce(() -> fineGrainDrive = !fineGrainDrive));

                new Trigger(() -> fineGrainDrive)
                                .onTrue(Commands.runOnce(() -> candle.addState(CandleState.FINE_GRAIN_DRIVE)))
                                .onFalse(Commands.runOnce(() -> candle.removeState(CandleState.FINE_GRAIN_DRIVE)));

                // brakes swerve, puts modules into x configuration
                driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));

                // manual module direction control
                driveController.b().whileTrue(drivetrain.applyRequest(() -> point
                                .withModuleDirection(new Rotation2d(-driveController.getLeftY(),
                                                -driveController.getLeftX()))));

                // reset the field-centric heading on left bumper press
                driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                driveController.rightTrigger().whileTrue(drivetrain.alignCommand(
                                Field.getTag(18).getLocation().toPose2d().transformBy(
                                                new Transform2d(PARTsUnit.InchesToMeters.apply(13.5 + 4),
                                                                0, new Rotation2d(PARTsUnit.DegreesToRadians
                                                                                .apply(-180.0))))));

                driveController.leftTrigger().whileTrue(drivetrain.alignCommand(
                                Field.getTag(12).getLocation().toPose2d().transformBy(
                                                new Transform2d(PARTsUnit.InchesToMeters.apply(13.5 + 4),
                                                                0, new Rotation2d(PARTsUnit.DegreesToRadians
                                                                                .apply(0.0))))));

                // logging
                drivetrain.registerTelemetry(telemetryLogger::telemeterize);

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                /*
                 * 
                 * driveController.back().and(driveController.y()).whileTrue(drivetrain.
                 * sysIdDynamic(Direction.kForward));
                 * driveController.back().and(driveController.x()).whileTrue(drivetrain.
                 * sysIdDynamic(Direction.kReverse));
                 * driveController.start().and(driveController.y()).whileTrue(drivetrain.
                 * sysIdQuasistatic(Direction.kForward));
                 * driveController.start().and(driveController.x()).whileTrue(drivetrain.
                 * sysIdQuasistatic(Direction.kReverse));
                 */

                // * */
                // =============================================================================================
                // * */ ------------------------------------- Elevator
                // * */ -------------------------------------------
                // * */
                // ---------------------------------------------------------------------------------------------

                operatorController.axisMagnitudeGreaterThan(5, 0.1)
                                .onTrue(elevator.joystickElevatorControl(operatorController));

                // =============================================================================================
                // * */ ------------------------------------- Coral Intake
                // * */ -------------------------------------------
                // * */
                // ---------------------------------------------------------------------------------------------

                buttonBoxController.positive4Trigger().onTrue(coral.intake()).onFalse(coral.stopCoralCommand());
                buttonBoxController.negative4Trigger().onTrue(coral.reverse()).onFalse(coral.stopCoralCommand());

                buttonBoxController.povTrigger0().whileTrue(coral.L4Intake()).onFalse(coral.stopCoralCommand());

                buttonBoxController.povTrigger180().whileTrue(coral.L4OutTake()).onFalse(coral.stopCoralCommand());
                operatorController.rightBumper().onTrue(new L4ScoreCoral(drivetrain, elevator, coral, candle));

                // =============================================================================================
                // * */ ------------------------------------- Elevator and Score Control
                // * */ -------------------------------------------
                // * */
                // ---------------------------------------------------------------------------------------------

                buttonBoxController.nukeTrigger().onTrue(Commands.runOnce(() -> {
                        elevatorManualControl = !elevatorManualControl;
                }));

                buttonBoxController.enginestartTrigger().onTrue(coral.score());

                // --------------------- Stow --------------------//
                buttonBoxController.negative1Trigger().onTrue(elevator.goToElevatorStow());

                // *---------------------------------------------------- *//
                // * ---------------Right Reef Pole Controls ----------- *//
                // *-----------------------------------------------------*//

                // --------------------- Align, L2, Score --------------------//
                /*
                 * buttonBoxController.mapTrigger()
                 * .onTrue(new ConditionalAlign(manualElevatorControlSupplier, elevator,
                 * ElevatorState.L2,
                 * drivetrain, coral, candle, buttonBoxController,
                 * new Pose2d(Constants.Drivetrain.xZeroHoldDistance
                 * .to(PARTsUnitType.Meter),
                 * Constants.Drivetrain.rightAlignDistance
                 * .to(PARTsUnitType.Meter),
                 * new Rotation2d()),
                 * frontVision, escapeBooleanSupplier));
                 * 
                 * // --------------------- Align, L3, Score --------------------//
                 * buttonBoxController.audioTrigger().onTrue(
                 * new ConditionalAlign(manualElevatorControlSupplier, elevator,
                 * ElevatorState.L3,
                 * drivetrain, coral, candle, buttonBoxController,
                 * new Pose2d(Constants.Drivetrain.xZeroHoldDistance
                 * .to(PARTsUnitType.Meter),
                 * Constants.Drivetrain.rightAlignDistance
                 * .to(PARTsUnitType.Meter),
                 * new Rotation2d()),
                 * frontVision, escapeBooleanSupplier));
                 * 
                 * // --------------------- Align, L4, Score --------------------//
                 * buttonBoxController.cruiseTrigger()
                 * .onTrue(new ConditionalAlign(manualElevatorControlSupplier, elevator,
                 * ElevatorState.L4,
                 * drivetrain, coral, candle, buttonBoxController,
                 * new Pose2d(Constants.Drivetrain.L4XDistance.to(PARTsUnitType.Meter),
                 * Constants.Drivetrain.rightAlignDistance
                 * .to(PARTsUnitType.Meter),
                 * new Rotation2d()),
                 * frontVision, escapeBooleanSupplier));
                 */

                // *---------------------------------------------------- *//
                // * ---------------Left Reef Pole Controls ----------- *//
                // *-----------------------------------------------------*//

                // --------------------- Align, L2, Score --------------------//
                /*
                 * buttonBoxController.wipeTrigger()
                 * .onTrue(new ConditionalAlign(manualElevatorControlSupplier, elevator,
                 * ElevatorState.L2,
                 * drivetrain, coral, candle, buttonBoxController,
                 * new Pose2d(Constants.Drivetrain.xZeroHoldDistance
                 * .to(PARTsUnitType.Meter),
                 * Constants.Drivetrain.leftAlignDistance
                 * .to(PARTsUnitType.Meter),
                 * new Rotation2d()),
                 * frontVision, escapeBooleanSupplier));
                 * 
                 * // --------------------- Align, L3, Score --------------------//
                 * buttonBoxController.flashTrigger()
                 * .onTrue(new ConditionalAlign(manualElevatorControlSupplier, elevator,
                 * ElevatorState.L3,
                 * drivetrain, coral, candle, buttonBoxController,
                 * new Pose2d(Constants.Drivetrain.xZeroHoldDistance
                 * .to(PARTsUnitType.Meter),
                 * Constants.Drivetrain.leftAlignDistance
                 * .to(PARTsUnitType.Meter),
                 * new Rotation2d()),
                 * frontVision, escapeBooleanSupplier));
                 * 
                 * // --------------------- Align, L4, Score --------------------//
                 * buttonBoxController.handleTrigger()
                 * .onTrue(new ConditionalAlign(manualElevatorControlSupplier, elevator,
                 * ElevatorState.L4,
                 * drivetrain, coral, candle, buttonBoxController,
                 * new Pose2d(Constants.Drivetrain.L4XDistance.to(PARTsUnitType.Meter),
                 * Constants.Drivetrain.leftAlignDistance
                 * .to(PARTsUnitType.Meter),
                 * new Rotation2d()),
                 * frontVision, escapeBooleanSupplier));
                 */

                // =============================================================================================
                // ------------------------------------- SysID
                // -------------------------------------------
                // ---------------------------------------------------------------------------------------------

                /*
                 * operatorController.a().and(operatorController.rightBumper())
                 * .whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                 * operatorController.b().and(operatorController.rightBumper())
                 * .whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                 * operatorController.x().and(operatorController.rightBumper())
                 * .whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
                 * operatorController.y().and(operatorController.rightBumper())
                 * .whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
                 */
                /*
                 * operatorController.a().and(operatorController.rightBumper())
                 * .whileTrue(algae.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                 * operatorController.b().and(operatorController.rightBumper())
                 * .whileTrue(algae.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                 * operatorController.x().and(operatorController.rightBumper())
                 * .whileTrue(algae.sysIdDynamic(SysIdRoutine.Direction.kForward));
                 * operatorController.y().and(operatorController.rightBumper())
                 * .whileTrue(algae.sysIdDynamic(SysIdRoutine.Direction.kReverse));
                 */
        }

        /*
         * public void configureAutonomousCommands() {
         * NamedCommands.registerCommand("Elevator L2",
         * elevator.elevatorToLevelCommand(ElevatorState.L2));
         * NamedCommands.registerCommand("Elevator Stow",
         * elevator.elevatorToLevelCommand(ElevatorState.STOW));
         * NamedCommands.registerCommand("Elevator L4",
         * elevator.elevatorToLevelCommand(ElevatorState.L4));
         * 
         * NamedCommands.registerCommand("Intake", coral.autoIntake());
         * NamedCommands.registerCommand("Score", coral.autoScore());
         * 
         * NamedCommands.registerCommand("Right Align L2 Score",
         * new AutoAlignScoreCoral(new Pose2d(
         * Constants.Drivetrain.xZeroHoldDistance.to(PARTsUnitType.Meter),
         * Constants.Drivetrain.rightAlignDistance
         * .to(PARTsUnitType.Meter),
         * new Rotation2d()), ElevatorState.L2, drivetrain, elevator, coral,
         * candle, frontVision));
         * NamedCommands.registerCommand("Left Align L2 Score",
         * new AutoAlignScoreCoral(new Pose2d(
         * Constants.Drivetrain.xZeroHoldDistance.to(PARTsUnitType.Meter),
         * Constants.Drivetrain.leftAlignDistance
         * .to(PARTsUnitType.Meter),
         * new Rotation2d()), ElevatorState.L2, drivetrain, elevator, coral,
         * candle, frontVision));
         * 
         * NamedCommands.registerCommand("Right Align L4 Score",
         * new AutoAlignScoreCoralL4(
         * new Pose2d(Constants.Drivetrain.L4XDistance.to(PARTsUnitType.Meter),
         * Constants.Drivetrain.rightAlignDistance
         * .to(PARTsUnitType.Meter),
         * new Rotation2d()),
         * ElevatorState.L4, drivetrain, elevator, coral, candle,
         * frontVision));
         * NamedCommands.registerCommand("Left Align L4 Score",
         * new AutoAlignScoreCoralL4(
         * new Pose2d(Constants.Drivetrain.L4XDistance.to(PARTsUnitType.Meter),
         * Constants.Drivetrain.leftAlignDistance
         * .to(PARTsUnitType.Meter),
         * new Rotation2d()),
         * ElevatorState.L4, drivetrain, elevator, coral, candle,
         * frontVision));
         * 
         * NamedCommands.registerCommand("Align Left L4 Stop",
         * new AutoAlignCoralStop(
         * new Pose2d(Constants.Drivetrain.L4XDistance.to(PARTsUnitType.Meter),
         * Constants.Drivetrain.leftAlignDistance
         * .to(PARTsUnitType.Meter),
         * new Rotation2d()),
         * ElevatorState.L4, drivetrain, elevator, coral, candle,
         * frontVision));
         * 
         * NamedCommands.registerCommand("Align Right L4 Stop",
         * new AutoAlignCoralStop(
         * new Pose2d(Constants.Drivetrain.L4XDistance.to(PARTsUnitType.Meter),
         * Constants.Drivetrain.rightAlignDistance
         * .to(PARTsUnitType.Meter),
         * new Rotation2d()),
         * ElevatorState.L4, drivetrain, elevator, coral, candle,
         * frontVision));
         * 
         * NamedCommands.registerCommand("Align Middle L1 Score",
         * new AutoAlignScoreCoral(
         * new Pose2d(Constants.Drivetrain.xZeroHoldDistance.to(PARTsUnitType.Meter),
         * 0,
         * new Rotation2d()),
         * ElevatorState.STOW, drivetrain, elevator, coral, candle,
         * frontVision));
         * 
         * NamedCommands.registerCommand("Align L1",
         * drivetrain.alignCommand(new
         * Pose2d(Constants.Drivetrain.xZeroHoldDistance.to(PARTsUnitType.Meter),
         * 0,
         * new Rotation2d()), frontVision));
         * 
         * autoChooser = AutoBuilder.buildAutoChooser();
         * SmartDashboard.putData("Auto Chooser", autoChooser);
         * }
         */

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public void outputTelemetry() {
                subsystems.forEach(s -> s.outputTelemetry());
                partsNT.setBoolean("Manual Mode", elevatorManualControl);
                partsNT.setDouble("Battery Voltage", RobotController.getBatteryVoltage());
        }

        public void stop() {
                subsystems.forEach(s -> s.stop());
        }

        public void log() {
                subsystems.forEach(s -> s.log());
        }

        public void setCandleDisabledState() {
                candle.addState(CandleState.DISABLED);
        }

        public void setIdleCandleState() {
                candle.addState(CandleState.IDLE);
                candle.removeState(CandleState.DISABLED);
        }

        public void constructDashboard() {
                PARTsDashboard.setSubsystems(subsystems);
                PARTsDashboard.setCommandScheduler();
        }

        public void resetStartPose() {
                drivetrain.seedFieldCentric();
        }
}
