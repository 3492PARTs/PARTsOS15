package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.VisionConstants;
import frc.robot.cmds.algae.Dealgae;
import frc.robot.cmds.coral.AlignScoreCoral;
import frc.robot.cmds.coral.PARTsAlignScoreCoral;
import frc.robot.cmds.coral.ScoreCoral;
//import frc.robot.commands.algae.AlgaeWrist;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Candle;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Candle.CandleState;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.PARTsDrivetrain;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.sysid.AlgaeSysId;
import frc.robot.subsystems.sysid.ElevatorSysId;
import frc.robot.subsystems.testing.Buttonbox;
import frc.robot.util.IPARTsSubsystem;
import frc.robot.util.PARTsButtonBoxController;
import frc.robot.util.PARTsDashboard;
import frc.robot.util.PARTsNT;
import frc.robot.util.PARTsSubsystem;
import frc.robot.util.PARTsUnit;
import frc.robot.util.PARTsUnit.PARTsUnitType;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


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

        private final CommandXboxController driveController = new CommandXboxController(0);
        private final CommandXboxController operatorController = new CommandXboxController(1);
        private final PARTsButtonBoxController buttonBoxController = new PARTsButtonBoxController(2);

        private boolean elevatorManualControl = false;

        /** Subsystems */
        private final Vision visionSubsystem = new Vision(Constants.VisionConstants.DRIVETRAIN_LIMELIGHT,
                        new PARTsUnit(Constants.VisionConstants.LIMELIGHT_ANGLE, PARTsUnitType.Angle),
                        new PARTsUnit(Constants.VisionConstants.LIMELIGHT_LENS_HEIGHT, PARTsUnitType.Inch));

        public final Candle candle = new Candle();

        private final Elevator elevator = new Elevator(candle);
        // private final ElevatorSysId elevator = new ElevatorSysId();

        private final Algae algae = new Algae();
        // private final AlgaeSysId algae = new AlgaeSysId();

        private final Coral coral = new Coral(candle, elevator);

        public final PARTsDrivetrain drivetrain = new PARTsDrivetrain(visionSubsystem,
                        TunerConstants.DrivetrainConstants,
                        TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft,
                        TunerConstants.BackRight);

        private final Climber climber = new Climber();

        private final ArrayList<IPARTsSubsystem> subsystems = new ArrayList<IPARTsSubsystem>(
                        Arrays.asList(candle, coral, elevator, drivetrain, climber, algae));

        private SendableChooser<Command> autoChooser;

        private PARTsNT partsNT = new PARTsNT("RobotContainer");

        /** End Subsystems */

        /*
         * NetworkTableInstance inst = NetworkTableInstance.getDefault();
         * StringTopic strTopic = inst.getStringTopic("/Elastic/CANColorValues");
         * StringPublisher strPub = strTopic.publish(PubSubOption.sendAll(true));
         */

        public RobotContainer() {
                configureAutonomousCommands();
                configureBindings();
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
                        if (elevator.getElevatorPosition() > Constants.Elevator.L2Height)
                                limit *= 0.25;
                        else if (fineGrainDrive)
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
                driveController.rightBumper().onTrue(Commands.runOnce(() -> {
                        fineGrainDrive = !fineGrainDrive;
                        if (fineGrainDrive)
                                candle.addState(CandleState.FINE_GRAIN_DRIVE);
                        else
                                candle.removeState(CandleState.FINE_GRAIN_DRIVE);
                }));

                // brakes swerve, puts modules into x configuration
                driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));

                // manual module direction control
                driveController.b().whileTrue(drivetrain.applyRequest(() -> point
                                .withModuleDirection(new Rotation2d(-driveController.getLeftY(),
                                                -driveController.getLeftX()))));

                // reset the field-centric heading on left bumper press
                driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                // driveController.leftTrigger()
                // .whileTrue(drivetrain.alignCommand(new Pose2d(-1, 0, new Rotation2d()),
                // driveController));

                driveController.rightTrigger().whileTrue(new PARTsAlignScoreCoral(
                                new Pose2d(0, new PARTsUnit(-7, PARTsUnitType.Inch).to(PARTsUnitType.Meter),
                                                new Rotation2d()),
                                ElevatorState.L2,
                                drivetrain, elevator, coral, candle));
                                
                driveController.leftTrigger().whileTrue(elevator.elevatorToLevelCommand(ElevatorState.L2));

                // logging
                drivetrain.registerTelemetry(telemetryLogger::telemeterize);

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                /*
                
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

                operatorController.rightBumper().onTrue(Commands.runOnce(() -> coral.scoreL4(), coral)).onFalse(coral.stopCoralCommand());

                // =============================================================================================
                // * */ ------------------------------------- Elevator and Score Control
                // * */ -------------------------------------------
                // * */
                // ---------------------------------------------------------------------------------------------

                buttonBoxController.nukeTrigger().onTrue(Commands.runOnce(() -> {
                        elevatorManualControl = !elevatorManualControl;
                }));

                buttonBoxController.enginestartTrigger().onTrue(coral.score());

                buttonBoxController.escTrigger().onTrue(new Dealgae(ElevatorState.A1, elevator, algae));

                buttonBoxController.enterTrigger().onTrue(new Dealgae(ElevatorState.A2, elevator, algae));

                //---------------------  Stow --------------------//
                buttonBoxController.negative1Trigger().onTrue(elevator.goToElevatorStow());

                //*---------------------------------------------------- *//
                //* ---------------Right Reef Pole Controls ----------- *//
                //*-----------------------------------------------------*//

                //--------------------- Align, L2, Score --------------------//
                buttonBoxController.mapTrigger().onTrue(Commands.runOnce(() -> {
                        if (elevatorManualControl)
                                elevator.goToElevatorL2().schedule();

                        else
                                new AlignScoreCoral(
                                                new Pose2d(0, Constants.Drivetrain.rightAlignDistance
                                                                .to(PARTsUnitType.Meter),
                                                                new Rotation2d()),
                                                ElevatorState.L2, drivetrain, elevator, coral, candle).schedule();
                }));

                //--------------------- Align, L3, Score --------------------//
                buttonBoxController.audioTrigger().onTrue(Commands.runOnce(() -> {
                        if (elevatorManualControl)
                                elevator.goToElevatorL3().schedule();
                        else
                                new AlignScoreCoral(
                                                new Pose2d(0, Constants.Drivetrain.rightAlignDistance
                                                                .to(PARTsUnitType.Meter),
                                                                new Rotation2d()),
                                                ElevatorState.L3, drivetrain, elevator, coral, candle).schedule();
                }));

                //--------------------- Align, L4, Score --------------------//
                buttonBoxController.cruiseTrigger().onTrue(Commands.runOnce(() -> {
                        if (elevatorManualControl)
                                elevator.goToElevatorL4().schedule();
                        else
                                new AlignScoreCoral(
                                                new Pose2d(0, Constants.Drivetrain.rightAlignDistance
                                                                .to(PARTsUnitType.Meter),
                                                                new Rotation2d()),
                                                ElevatorState.L4, drivetrain, elevator, coral, candle).schedule();
                }));

                //*---------------------------------------------------- *//
                //* ---------------Left Reef Pole Controls ----------- *//
                //*-----------------------------------------------------*//

                //--------------------- Align, L2, Score --------------------//
                buttonBoxController.wipeTrigger().onTrue(Commands.runOnce(() -> {
                        if (elevatorManualControl)
                                elevator.goToElevatorL2().schedule();
                        else
                                new AlignScoreCoral(
                                                new Pose2d(0, Constants.Drivetrain.leftAlignDistance
                                                                .to(PARTsUnitType.Meter),
                                                                new Rotation2d()),
                                                ElevatorState.L2, drivetrain, elevator, coral, candle).schedule();
                }));

                //--------------------- Align, L3, Score --------------------//
                buttonBoxController.flashTrigger().onTrue(Commands.runOnce(() -> {
                        if (elevatorManualControl)
                                elevator.goToElevatorL3().schedule();
                        else
                                new AlignScoreCoral(
                                                new Pose2d(0, Constants.Drivetrain.leftAlignDistance
                                                                .to(PARTsUnitType.Meter),
                                                                new Rotation2d()),
                                                ElevatorState.L3, drivetrain, elevator, coral, candle).schedule();
                }));

                //--------------------- Align, L4, Score --------------------//
                buttonBoxController.handleTrigger().onTrue(Commands.runOnce(() -> {
                        if (elevatorManualControl)
                                elevator.goToElevatorL4().schedule();
                        else
                                new AlignScoreCoral(
                                                new Pose2d(0, Constants.Drivetrain.leftAlignDistance
                                                                .to(PARTsUnitType.Meter),
                                                                new Rotation2d()),
                                                ElevatorState.L4, drivetrain, elevator, coral, candle).schedule();
                }));

                // * */
                // =============================================================================================
                // * */ ------------------------------------- Algae Control
                // * */ -------------------------------------------
                // * */
                // ---------------------------------------------------------------------------------------------

                operatorController.povDown().onTrue(algae.grabReefAlgae());

                operatorController.povLeft().onTrue(algae.stow());
                
                operatorController.axisMagnitudeGreaterThan(1, 0.1)
                .onTrue(algae.joystickAlgaeControl(operatorController));

                operatorController.povRight().whileTrue(Commands.runOnce(() -> algae.reset()));
                
                operatorController.povUp().whileTrue(Commands.run(() -> algae.setIntakeSpeed(Constants.Algae.kReefIntakeSpeed))).whileFalse(Commands.runOnce(() -> algae.setIntakeSpeed(0)));

                //operatorController.povLeft().onTrue(algae.stopAlgae());
                //operatorController.leftTrigger().onTrue(Commands.runOnce(algae::reset));
                //operatorController.leftBumper().whileTrue(algae.score());

                // =============================================================================================
                // ------------------------------------- Climber
                // -------------------------------------------
                // ---------------------------------------------------------------------------------------------

                operatorController.povUp().whileTrue(Commands.run(() -> climber.setSpeed(-.3)))
                                .whileFalse(Commands.run(() -> climber.setSpeed(0)));
                operatorController.povDown().whileTrue(Commands.run(() -> climber.setSpeed(.3)))
                                .whileFalse(Commands.run(() -> climber.setSpeed(0)));
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

        public void configureAutonomousCommands() {   
                NamedCommands.registerCommand("Elevator L2", elevator.elevatorToLevelCommand(ElevatorState.L2));
                NamedCommands.registerCommand("Intake", coral.autoIntake());
                NamedCommands.registerCommand("Score", coral.autoScore());
                NamedCommands.registerCommand("Elevator Stow", elevator.elevatorToLevelCommand(ElevatorState.STOW));
                NamedCommands.registerCommand("Elevator L4", elevator.elevatorToLevelCommand(ElevatorState.L4));
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

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
                candle.disable();
        }

        public void setIdleCandleState() {
                candle.addState(CandleState.IDLE);
        }

        public void constructDashboard() {
                PARTsDashboard.setSubsystems(subsystems);
                PARTsDashboard.setCommandScheduler();
        }

        public void resetStartPose() {
                drivetrain.resetRotation(new Rotation2d(Math.PI));
        }
}
