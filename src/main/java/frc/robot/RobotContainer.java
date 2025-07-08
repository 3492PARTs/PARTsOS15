package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.BooleanSupplier;

import org.opencv.core.Mat;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.generated.TunerConstants;
import frc.robot.states.ElevatorState;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Candle.CandleState;
import frc.robot.subsystems.Coral.Coral;
import frc.robot.subsystems.Coral.CoralPhys;
import frc.robot.subsystems.Coral.CoralSim;
import frc.robot.subsystems.Drivetrain.PARTsDrivetrain;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPhys;
import frc.robot.subsystems.Elevator.ElevatorSim;
import frc.robot.util.Field.Field;
import frc.robot.util.Field.Reef;
import frc.robot.util.PARTs.PARTsButtonBoxController;
import frc.robot.util.PARTs.PARTsCommandController;
import frc.robot.util.PARTs.PARTsCommandUtils;
import frc.robot.util.PARTs.PARTsDashboard;
import frc.robot.util.PARTs.PARTsNT;
import frc.robot.util.PARTs.Interfaces.IPARTsSubsystem;
import frc.robot.util.PARTs.PARTsController.ControllerType;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.LimelightVision.MegaTagMode;

public class RobotContainer {

        private final PARTsCommandController driveController = new PARTsCommandController(0, ControllerType.XBOX);
        private final PARTsCommandController operatorController = new PARTsCommandController(1, ControllerType.XBOX);
        private final PARTsButtonBoxController buttonBoxController = new PARTsButtonBoxController(2);

        private final GenericHID keyboardController = Robot.isSimulation() ? new GenericHID(5) : null;

        private boolean visionAlignActive = true;
        private BooleanSupplier visionAlignActiveBooleanSupplier = () -> visionAlignActive;

        /* Subsystems */

        public final Candle candle = new Candle();

        private final Elevator elevator = Robot.isReal() ? new ElevatorPhys() : new ElevatorSim();
        // private final ElevatorSysId elevator = new ElevatorSysId();

        private final Coral coral = Robot.isReal() ? new CoralPhys(candle, elevator) : new CoralSim(candle, elevator);

        public final PARTsDrivetrain drivetrain = new PARTsDrivetrain(
                        TunerConstants.DrivetrainConstants,
                        TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft,
                        TunerConstants.BackRight);

        private final LimelightVision vision = new LimelightVision(drivetrain);

        private final ArrayList<IPARTsSubsystem> subsystems = new ArrayList<IPARTsSubsystem>(
                        Arrays.asList(candle, coral, elevator, drivetrain, vision));

        /** End Subsystems */

        private SendableChooser<Command> autoChooser;

        private PARTsNT partsNT = new PARTsNT("RobotContainer");

        private BooleanSupplier escapeBooleanSupplier;

        public RobotContainer() {
                escapeBooleanSupplier = buttonBoxController.negative3Trigger()
                                .or(driveController.axisMagnitudeGreaterThan(0, 0.5))
                                .or(driveController.axisMagnitudeGreaterThan(1, 0.5))
                                .or(driveController.rightTrigger());

                configureDrivetrainBindings();
                configureElevatorBindings();
                configureCoralBindings();

                partsNT.putSmartDashboardSendable("field", Field.FIELD2D);

                if (Robot.isSimulation()) {
                        drivetrain.resetPose(Field.conditionallyTransformToOppositeAlliance(new Pose2d(
                                        new Translation2d(Units.inchesToMeters(607.37), Units.inchesToMeters(291.20)),
                                        new Rotation2d())));
                        drivetrain.setOperatorPerspectiveForward(
                                        Robot.isBlue() ? new Rotation2d() : new Rotation2d(Math.PI));
                        // drivetrain.seedFieldCentric();
                }
        }

        private void configureDrivetrainBindings() {

                // Drivetrain will execute this command periodically
                drivetrain.setDefaultCommand(drivetrain.commandDefault(driveController));

                // fine grain controls
                // driveController.rightBumper().onTrue(Commands.runOnce(() -> fineGrainDrive =
                // !fineGrainDrive));

                // new Trigger(() -> fineGrainDrive)
                // .onTrue(Commands.runOnce(() ->
                // candle.addState(CandleState.FINE_GRAIN_DRIVE)))
                // .onFalse(Commands.runOnce(() ->
                // candle.removeState(CandleState.FINE_GRAIN_DRIVE)));

                // brakes swerve, puts modules into x configuration
                driveController.a().whileTrue(drivetrain.commandBrake());

                // manual module direction control
                driveController.b().whileTrue(drivetrain.commandPointWheels(driveController));

                // reset the field-centric heading on left bumper press
                driveController.leftBumper().onTrue(drivetrain.commandSeedFieldCentric());

                if (RobotConstants.debug) {
                        new Trigger(() -> keyboardController.getRawButton(1))
                                        .onTrue(Reef.commandIntakeScoreIntake(drivetrain, coral, elevator));

                        new Trigger(() -> keyboardController.getRawButton(2))
                                        .onTrue(drivetrain.commandPathOnTheFly(
                                                        Field.getTag(12).getLocation().toPose2d()));

                        driveController.rightTrigger()
                                        .whileTrue(drivetrain.commandPathOnTheFly(
                                                        Field.getTag(12).getLocation().toPose2d()));
                        // driveController.rightTrigger().whileTrue(Reef.commandIntakeScoreIntake(drivetrain,
                        // coral, elevator));
                        driveController.leftTrigger()
                                        .whileTrue(vision.commandMegaTagMode(MegaTagMode.MEGATAG2));
                }

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

        }

        private void configureElevatorBindings() {
                operatorController.axisMagnitudeGreaterThan(5, 0.1)
                                .onTrue(elevator.commandJoystickControl(operatorController));

                // --------------------- Stow --------------------//
                buttonBoxController.negative1Trigger().onTrue(elevator.commandStow());

                buttonBoxController.nukeTrigger().toggleOnTrue(
                                PARTsCommandUtils.setCommandName("Toggle Vision Control", Commands.startEnd(
                                                () -> {
                                                        visionAlignActive = true;
                                                },
                                                () -> {
                                                        visionAlignActive = false;
                                                })));

                // *---------------------------------------------------- *//
                // * ---------------Right Reef Pole Controls ----------- *//
                // *-----------------------------------------------------*//

                // --------------------- Align, L2, Score --------------------//
                buttonBoxController.mapTrigger()
                                .onTrue(Reef.commandAlignAndScoreToVisibleTag(true, drivetrain, elevator,
                                                ElevatorState.L2,
                                                coral,
                                                escapeBooleanSupplier, candle, visionAlignActiveBooleanSupplier));

                // --------------------- Align, L3, Score --------------------//
                buttonBoxController.audioTrigger()
                                .onTrue(Reef.commandAlignAndScoreToVisibleTag(true, drivetrain, elevator,
                                                ElevatorState.L3, coral, escapeBooleanSupplier, candle,
                                                visionAlignActiveBooleanSupplier));

                // --------------------- Align, L4, Score --------------------//
                buttonBoxController.cruiseTrigger()
                                .onTrue(Reef.commandAlignAndScoreToVisibleTag(true, drivetrain, elevator,
                                                ElevatorState.L4,
                                                coral,
                                                escapeBooleanSupplier, candle, visionAlignActiveBooleanSupplier));

                // *---------------------------------------------------- *//
                // * ---------------Left Reef Pole Controls ----------- *//
                // *-----------------------------------------------------*//

                // --------------------- Align, L2, Score --------------------//

                buttonBoxController.wipeTrigger()
                                .onTrue(Reef.commandAlignAndScoreToVisibleTag(false, drivetrain, elevator,
                                                ElevatorState.L2,
                                                coral,
                                                escapeBooleanSupplier, candle, visionAlignActiveBooleanSupplier));

                // --------------------- Align, L3, Score --------------------//
                buttonBoxController.flashTrigger()
                                .onTrue(Reef.commandAlignAndScoreToVisibleTag(false, drivetrain, elevator,
                                                ElevatorState.L3,
                                                coral,
                                                escapeBooleanSupplier, candle, visionAlignActiveBooleanSupplier));

                // --------------------- Align, L4, Score --------------------//
                buttonBoxController.handleTrigger()
                                .onTrue(Reef.commandAlignAndScoreToVisibleTag(false, drivetrain, elevator,
                                                ElevatorState.L4,
                                                coral,
                                                escapeBooleanSupplier, candle, visionAlignActiveBooleanSupplier));

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

                if (Robot.isSimulation()) {
                        operatorController.a().onTrue(elevator.commandL2());
                        operatorController.b().onTrue(elevator.commandL3());
                        operatorController.y().onTrue(elevator.commandL4());
                        operatorController.x().onTrue(elevator.commandStow());
                }
        }

        private void configureCoralBindings() {
                buttonBoxController.positive4Trigger().onTrue(coral.commandIntake()).onFalse(coral.commandStop());

                buttonBoxController.negative4Trigger().onTrue(coral.commandReverse()).onFalse(coral.commandStop());

                buttonBoxController.povTrigger0().whileTrue(coral.commandL4Intake()).onFalse(coral.commandStop());

                buttonBoxController.povTrigger180().whileTrue(coral.commandL4OutTake())
                                .onFalse(coral.commandStop());

                buttonBoxController.enginestartTrigger().onTrue(coral.commandScore());
        }

        public void configureAutonomousCommands() {
                NamedCommands.registerCommand("Elevator L2",
                                elevator.commandToLevel(ElevatorState.L2));
                NamedCommands.registerCommand("Elevator Stow",
                                elevator.commandToLevel(ElevatorState.STOW));
                NamedCommands.registerCommand("Elevator L4",
                                elevator.commandToLevel(ElevatorState.L4));

                NamedCommands.registerCommand("Intake", coral.commandAutoIntake());
                NamedCommands.registerCommand("Score", coral.autoScore());

                NamedCommands.registerCommand("Right Align L2 Score",
                                Reef.commandAlignAndScoreToVisibleTag(true, drivetrain, elevator, ElevatorState.L2,
                                                coral,
                                                escapeBooleanSupplier, candle, visionAlignActiveBooleanSupplier));
                NamedCommands.registerCommand("Left Align L2 Score",
                                Reef.commandAlignAndScoreToVisibleTag(false, drivetrain, elevator, ElevatorState.L2,
                                                coral,
                                                escapeBooleanSupplier, candle, visionAlignActiveBooleanSupplier));

                NamedCommands.registerCommand("Right Align L4 Score",
                                Reef.commandAlignAndScoreToVisibleTag(true, drivetrain, elevator, ElevatorState.L4,
                                                coral,
                                                escapeBooleanSupplier, candle, visionAlignActiveBooleanSupplier));
                NamedCommands.registerCommand("Left Align L4 Score",
                                Reef.commandAlignAndScoreToVisibleTag(false, drivetrain, elevator, ElevatorState.L4,
                                                coral,
                                                escapeBooleanSupplier, candle, visionAlignActiveBooleanSupplier));

                NamedCommands.registerCommand("Align Left L4 Stop",
                                Reef.alignToVisibleTagSideStop(false, drivetrain, elevator, ElevatorState.L4, coral,
                                                escapeBooleanSupplier, candle));

                NamedCommands.registerCommand("Align Right L4 Stop",
                                Reef.alignToVisibleTagSideStop(true, drivetrain, elevator, ElevatorState.L4, coral,
                                                escapeBooleanSupplier, candle));

                /*
                 * NamedCommands.registerCommand("Align Middle L1 Score",
                 * new AutoAlignScoreCoral(
                 * new Pose2d(Constants.Drivetrain.xZeroHoldDistance
                 * .to(PARTsUnitType.Meter),
                 * 0,
                 * new Rotation2d()),
                 * ElevatorState.STOW, drivetrain, elevator, coral, candle,
                 * frontVision));
                 * 
                 * NamedCommands.registerCommand("Align L1",
                 * drivetrain.alignCommand(new Pose2d(
                 * Constants.Drivetrain.xZeroHoldDistance.to(PARTsUnitType.Meter),
                 * 0,
                 * new Rotation2d()), frontVision));
                 */

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public void outputTelemetry() {
                subsystems.forEach(s -> s.outputTelemetry());
                partsNT.putBoolean("Vision Mode", visionAlignActive);
                partsNT.putDouble("Battery Voltage", RobotController.getBatteryVoltage());
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

        public void setMegaTagMode(MegaTagMode mode) {
                vision.setMegaTagMode(mode);
        }

}
