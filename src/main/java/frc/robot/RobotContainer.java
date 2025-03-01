// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Candle;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Candle.CandleState;
import frc.robot.subsystems.sysid.AlgaeSysId;
import frc.robot.subsystems.sysid.ElevatorSysId;
import frc.robot.util.IPARTsSubsystem;
import frc.robot.util.PARTsDashboard;
import frc.robot.util.PARTsUnit;
import frc.robot.util.PARTsUnit.PARTsUnitType;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.PARTsDrivetrain;

public class RobotContainer {
    private boolean fineGrainDrive = false;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry telemetryLogger = new Telemetry(MaxSpeed);

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    /**Subsystems */
    private final Vision visionSubsystem = new Vision(VisionConstants.DRIVETRAIN_LIMELIGHT,
            new PARTsUnit(VisionConstants.LIMELIGHT_ANGLE, PARTsUnitType.Angle),
            new PARTsUnit(VisionConstants.LIMELIGHT_LENS_HEIGHT, PARTsUnitType.Inch));

    public final Candle candle = new Candle();

    private final Elevator elevator = new Elevator(candle);
    //private final ElevatorSysId elevator = new ElevatorSysId();

    //private final Algae algae = new Algae();
    //private final AlgaeSysId algae = new AlgaeSysId();

    private final Coral coral = new Coral(candle, elevator);

    public final PARTsDrivetrain drivetrain = new PARTsDrivetrain(visionSubsystem, TunerConstants.DrivetrainConstants,
            TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);

    //TODO: add algae to list later
    private final ArrayList<IPARTsSubsystem> subsystems = new ArrayList<IPARTsSubsystem>(
            Arrays.asList(candle, coral, elevator, drivetrain));

    

    /**End Subsystems */

    /*
     * NetworkTableInstance inst = NetworkTableInstance.getDefault();
     * StringTopic strTopic = inst.getStringTopic("/Elastic/CANColorValues");
     * StringPublisher strPub = strTopic.publish(PubSubOption.sendAll(true));
     */

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        //* */ =============================================================================================
        //* */ ------------------------------------- DriveTrain
        //* */ -------------------------------------------
        //* */ ---------------------------------------------------------------------------------------------

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        Command driveCommand = drivetrain.applyRequest(() -> {
            double limit = MaxSpeed;
            if (elevator.getElevatorPosition() > Constants.Elevator.L2Height)
                limit = 0.5;
            else if (fineGrainDrive)
                limit = 0.5;
            return drive.withVelocityX(-driveController.getLeftY() * limit) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveController.getLeftX() * limit) // Drive left with negative X (left)
                    .withRotationalRate(-driveController.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
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
                .withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))));

        // reset the field-centric heading on left bumper press
        driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driveController.leftTrigger()
                .whileTrue(drivetrain.alignCommand(new Pose2d(-1, 0, new Rotation2d()), driveController));

        // logging
        drivetrain.registerTelemetry(telemetryLogger::telemeterize);

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        /* 
        driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        */

        //* */ =============================================================================================
        //* */ ------------------------------------- Elevator
        //* */ -------------------------------------------
        //* */ ---------------------------------------------------------------------------------------------
        /*
        elevator.setDefaultCommand(new RunCommand(() -> {
            double speed = -operatorController.getRightY() * Constants.Elevator.maxSpeed;
        
            if (Math.abs(speed) < 0.1)
                speed = 0;
            elevator.setElevatorPower(speed);
        }, elevator));*/

        // While the joystick is moving control the elevator in manual, and when done stop
        operatorController.axisMagnitudeGreaterThan(5, 0.1)
                .onTrue(elevator.joystickElevatorControl(operatorController));

        operatorController.a().onTrue(elevator.goToElevatorStow());

        operatorController.x().onTrue(elevator.goToElevatorL2());

        operatorController.y().onTrue(elevator.goToElevatorL3());

        operatorController.b().onTrue(elevator.goToElevatorL4());

        //* */ =============================================================================================
        //* */ ------------------------------------- Coral Controls
        //* */ -------------------------------------------
        //* */ ---------------------------------------------------------------------------------------------

        operatorController.rightTrigger().onTrue(coral.intake());
        operatorController.rightBumper().onTrue(coral.reverse());
        operatorController.leftTrigger().onTrue(coral.stopCoral());
        operatorController.leftBumper().onTrue(coral.scoreCommand());

        //* */ =============================================================================================
        //* */ ------------------------------------- Algae Control
        //* */ -------------------------------------------
        //* */ ---------------------------------------------------------------------------------------------

        //operatorController.leftBumper().whileTrue(new AlgaeIntake(algae, operatorController));
        // operatorController.leftTrigger().whileTrue(getAutonomousCommand()));
        //algae.setDefaultCommand(new AlgaeWrist(algae, operatorController));

        //TODO: Please migrate from run command, example Elevator.java - public Command goToElevatorL4()
        // TODO: We are migrating to command factory structure. (i.e. creating and using a command though a function call)
        /* 
        operatorController.povUp().onTrue(algae.stow());
        operatorController.povDown().onTrue(algae.grabReefAlgae());
        operatorController.povRight().onTrue(algae.groundIntake());
        operatorController.povLeft().onTrue(algae.stopAlgae());
        operatorController.leftTrigger().onTrue(Commands.runOnce(algae::reset));
        operatorController.leftBumper().whileTrue(algae.score());
         

        // =============================================================================================
        // ------------------------------------- SysID
        // -------------------------------------------
        // ---------------------------------------------------------------------------------------------

        /*  
         operatorController.a().and(operatorController.rightBumper())
         .whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
         operatorController.b().and(operatorController.rightBumper())
         .whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
         operatorController.x().and(operatorController.rightBumper())
         .whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
         operatorController.y().and(operatorController.rightBumper())
         .whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        */
        /* 
        operatorController.a().and(operatorController.rightBumper())
        .whileTrue(algae.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        operatorController.b().and(operatorController.rightBumper())
        .whileTrue(algae.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        operatorController.x().and(operatorController.rightBumper())
        .whileTrue(algae.sysIdDynamic(SysIdRoutine.Direction.kForward));
        operatorController.y().and(operatorController.rightBumper())
        .whileTrue(algae.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        */
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void outputTelemetry() {
        subsystems.forEach(s -> s.outputTelemetry());
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
}
