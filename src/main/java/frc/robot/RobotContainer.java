// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.VisionConstants;
import frc.robot.Commands.AlignCommand;
import frc.robot.Commands.DriveLogCommand;
import frc.robot.Commands.Algae.AlgaeIntake;
import frc.robot.Commands.Algae.AlgaeWrist;
import frc.robot.Commands.Coral.CoralAction;
import frc.robot.Commands.ElevatorCommands.ElevatorJoystick;
import frc.robot.Commands.ElevatorCommands.ZeroElevatorEncoderCmdSeq;
import frc.robot.subsystems.Algae;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.util.PARTsUnit;
import frc.robot.util.PARTsUnit.PARTsUnitType;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Vision visionSubsystem = new Vision(VisionConstants.DRIVETRAIN_LIMELIGHT, new PARTsUnit(VisionConstants.LIMELIGHT_ANGLE, PARTsUnitType.Angle), new PARTsUnit(VisionConstants.LIMELIGHT_LENS_HEIGHT, PARTsUnitType.Inch));

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final Elevator elevator = new Elevator();

    public final Trigger zeroElevatorTrigger = new Trigger(elevator.getLimitSwitchSupplier());

  //  private final Algae algae = new Algae();

    private final Coral coral = new Coral();

    private final CommandXboxController driveController = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
        DataLogManager.start();
    }

    private void configureBindings() {
        // =============================================================================================
        // ------------------------------------- DriveTrain
        // -------------------------------------------
        // ---------------------------------------------------------------------------------------------

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-driveController.getLeftY() * MaxSpeed) // Drive
                                                                                                          // forward
                                                                                                          // with
                                                                                                          // negative Y
                                                                                                          // (forward)
                        .withVelocityY(-driveController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-driveController.getRightX() * MaxAngularRate) // Drive counterclockwise
                                                                                           // with negative X (left)
                ));

        // brakes swerve, puts modules into x configuration
        driveController.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // manual module direction control
        driveController.b().whileTrue(drivetrain.applyRequest(() -> point
                .withModuleDirection(new Rotation2d(-driveController.getLeftY(), -driveController.getLeftX()))));

        // reset the field-centric heading on left bumper press
        driveController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        driveController.leftTrigger().onTrue(new AlignCommand(
            visionSubsystem, 
            drivetrain, 
            new Pose2d(-1,0, new Rotation2d())
            ));
            
        // logging
        drivetrain.registerTelemetry(logger::telemeterize);

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driveController.back().and(driveController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driveController.back().and(driveController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driveController.start().and(driveController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driveController.start().and(driveController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // =============================================================================================
        // ------------------------------------- Elevator
        // -------------------------------------------
        // ---------------------------------------------------------------------------------------------

        elevator.setDefaultCommand(new ElevatorJoystick(elevator, operatorController));

        operatorController.a().onTrue(new RunCommand(() -> {
            elevator.goToElevatorStow();
        }, elevator));

        operatorController.x().onTrue(new RunCommand(() -> {
            elevator.goToElevatorL2();
        }, elevator));

        operatorController.y().onTrue(new RunCommand(() -> {
            elevator.goToElevatorL3();
        }, elevator));

        operatorController.b().onTrue(new RunCommand(() -> {
            elevator.goToElevatorL4();
          }, elevator));


        /*
         * if (operatorController.getWantsElevatorStow()) {
         * m_elevator.goToElevatorStow();
         * m_algae.stow();
         * } else if (operatorController.getWantsElevatorL2()) {
         * m_elevator.goToElevatorL2();
         * m_algae.stow();
         * } else if (operatorController.getWantsElevatorL3()) {
         * m_elevator.goToElevatorL3();
         * m_algae.stow();
         * } else if (m_operatorController.getWantsElevatorL4()) {
         * m_elevator.goToElevatorL4();
         * m_algae.stow();
         * }
         */

        // zeros elevator encoders
        zeroElevatorTrigger.onTrue(new ZeroElevatorEncoderCmdSeq(elevator));

        // =============================================================================================
        // ------------------------------------- Coral Controls
        // -------------------------------------------
        // ---------------------------------------------------------------------------------------------

        operatorController.b().whileTrue(new CoralAction(coral, operatorController));

        // =============================================================================================
        // ------------------------------------- Algae Controls
        // -------------------------------------------
        // ---------------------------------------------------------------------------------------------

      //  operatorController.x().whileTrue(new AlgaeIntake(algae, operatorController));
       // operatorController.a().whileTrue(new AlgaeWrist(algae, operatorController));

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
