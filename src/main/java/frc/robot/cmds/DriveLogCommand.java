// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.cmds;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.generated.TunerConstants;
import frc.robot.subsystems.PARTsDrivetrain;
import frc.robot.util.PARTsUnit;
import frc.robot.util.PARTsUnit.PARTsUnitType;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveLogCommand extends Command {

  public boolean isFinished = false;

  double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  final PARTsDrivetrain drivetrain;
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  final CommandXboxController joystick;

  // PID for driving.
  private static final double RANGE_P = 0.8;
  private static final double RANGE_I = 0.04;
  private static final double RANGE_D = 0.1;

  // PID Controllers
  // private final ProfiledPIDController thetaController;
  private final ProfiledPIDController xRangeController;
  private final ProfiledPIDController yRangeController;

  private final SwerveDrivePoseEstimator m_poseEstimator;

  // * Log Data
  // init robot
  DoubleLogEntry logInitRobotX;
  DoubleLogEntry logInitRobotY;
  DoubleLogEntry logInitRobotZ;

  // init limelight
  DoubleLogEntry logInitLimeX;
  DoubleLogEntry logInitLimeY;
  DoubleLogEntry logInitLimeZ;

  // curr robot
  DoubleLogEntry logCurrRobotX;
  DoubleLogEntry logCurrRobotY;
  DoubleLogEntry logCurrRobotZ;

  // curr limelight
  DoubleLogEntry logCurrLimeX;
  DoubleLogEntry logCurrLimeY;
  DoubleLogEntry logCurrLimeZ;

  // curr post calc robot
  DoubleLogEntry logCurrRobotCalcX;
  DoubleLogEntry logCurrRobotCalcY;

  // curr post calc limelight
  DoubleLogEntry logCurrLimeCalcX;
  DoubleLogEntry logCurrLimeCalcY;

  // Curr rotation LL
  DoubleLogEntry logCurrLLRotationX;
  DoubleLogEntry logCurrLLRotationY;
  DoubleLogEntry logCurrLLRotationZ;

  // Curr rotation robot
  DoubleLogEntry logCurrRobotRotationX;
  DoubleLogEntry logCurrRobotRotationY;

  boolean doRejectUpdate = false;

  /** Creates a new DriveLog. */
  public DriveLogCommand(PARTsDrivetrain drivetrain, CommandXboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.joystick = joystick;

    addRequirements(drivetrain);

    xRangeController = new ProfiledPIDController(RANGE_P, RANGE_I, RANGE_D, new TrapezoidProfile.Constraints(1.0, 0.5));
    yRangeController = new ProfiledPIDController(RANGE_P, RANGE_I, RANGE_D, new TrapezoidProfile.Constraints(1.0, 0.5));

    DataLog log = DataLogManager.getLog();

    m_poseEstimator = new SwerveDrivePoseEstimator(
        drivetrain.getKinematics(),
        drivetrain.getRotation3d().toRotation2d(),
        new SwerveModulePosition[] {
            drivetrain.getModule(0).getPosition(doRejectUpdate),
            drivetrain.getModule(1).getPosition(doRejectUpdate),
            drivetrain.getModule(2).getPosition(doRejectUpdate),
            drivetrain.getModule(3).getPosition(doRejectUpdate)
        },
        new Pose2d(),
        VecBuilder.fill(0.05, 0.05, new PARTsUnit(5, PARTsUnitType.Angle).to(PARTsUnitType.Radian)),
        VecBuilder.fill(0.5, 0.5, new PARTsUnit(30, PARTsUnitType.Angle).to(PARTsUnitType.Radian)));

    // Initial Robot
    // logInitRobotX= new DoubleLogEntry(log, "/PARTs/log/initRobotX");
    // logInitRobotY = new DoubleLogEntry(log, "/PARTs/log/initRobotY");

    // Initial LL
    // logInitLimeX = new DoubleLogEntry(log, "/PARTs/log/initLimeX");
    // logInitLimeY = new DoubleLogEntry(log, "/PARTs/log/initLimeY");
    // logInitLimeZ = new DoubleLogEntry(log, "/PARTs/log/initLimeZ");

    // Current Robot
    // logCurrRobotX = new DoubleLogEntry(log, "/PARTs/log/currRobotX");
    // logCurrRobotY = new DoubleLogEntry(log, "/PARTs/log/currRobotY");

    // Current LL
    // logCurrLimeX = new DoubleLogEntry(log, "/PARTs/log/currLimeX");
    // logCurrLimeY = new DoubleLogEntry(log, "/PARTs/log/currLimeY");
    // logCurrLimeZ = new DoubleLogEntry(log, "/PARTs/log/currLimeZ");

    // Calculated
    // logCurrRobotCalcX = new DoubleLogEntry(log, "/PARTs/log/currRobotCalcX");
    // logCurrRobotCalcY = new DoubleLogEntry(log, "/PARTs/log/currRobotCalcY");
    // logCurrLimeCalcX = new DoubleLogEntry(log, "/PARTs/log/currLimeCalcX");
    // logCurrLimeCalcY = new DoubleLogEntry(log, "/PARTs/log/currLimeCalcY");

    // Current LL Rotation
    logCurrLLRotationX = new DoubleLogEntry(log, "/PARTs/log/currLLXRotation");
    logCurrLLRotationY = new DoubleLogEntry(log, "/PARTs/log/currLLYRotation");
    logCurrLLRotationZ = new DoubleLogEntry(log, "/PARTs/log/currLLZRotation");

    // Current Robot Rotation
    logCurrRobotRotationX = new DoubleLogEntry(log, "/PARTs/log/currRobotXRotation");
    logCurrRobotRotationY = new DoubleLogEntry(log, "/PARTs/log/currRobotYRotation");
    // logCurrRobotRotationZ = new DoubleLogEntry(log,
    // "/PARTs/log/currRobotZRotation");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // initialRobotPose3d = convertToRobotSpace(m_Vision.getPose3d());

    drivetrain.resetPose(new Pose2d(0, 0, new Rotation2d(0)));
    System.out.println(
        "Drive Log Init\nCurrent rotation after reset: " + drivetrain.getRotation3d().toRotation2d().getRadians());

    // logInitRobotX.append(drivetrain.getState().Pose.getX());
    // logInitRobotY.append(drivetrain.getState().Pose.getY());

    // logInitLimeZ.append(vision.getPose3d().getZ());
    // logInitLimeX.append(vision.getPose3d().getX());
    // logInitLimeY.append(vision.getPose3d().getY());

    // Initialize the x-range controller.
    // xRangeController.reset(initialRobotPose3d.getX());
    // xRangeController.setGoal(holdDistance.getX());
    // xRangeController.setTolerance(0.1);

    // Initialize the y-range controller.
    // yRangeController.reset(initialRobotPose3d.getY()); // Center to target.
    // yRangeController.setGoal(holdDistance.getY()); // Center to target.
    // yRangeController.setTolerance(0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rawX = (-joystick.getLeftY() * MaxSpeed);
    double rawY = (-joystick.getLeftX() * MaxSpeed);
    double rawR = (-joystick.getRightX() * MaxAngularRate);

    // Log raw values.
    // logCurrLimeX.append(vision.getPose3d().getX());
    // logCurrLimeY.append(vision.getPose3d().getY());
    // logCurrLimeZ.append(vision.getPose3d().getZ());

    // logCurrRobotX.append(drivetrain.getState().Pose.getX());
    // logCurrRobotY.append(drivetrain.getState().Pose.getY());

    //drivetrain.updatePoseEstimator();

    // logCurrLLRotationX.append(m_poseEstimator.getEstimatedPosition().getX());
    // logCurrLLRotationY.append(m_poseEstimator.getEstimatedPosition().getY());
    // logCurrLLRotationZ.append(m_poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    // System.out.println("TX: " + LimelightHelpers.getTX(""));
    // System.out.println("TY: " + LimelightHelpers.getTY(""));

    // logCurrLLRotationZ.append(LimelightHelpers.getBotPose3d_TargetSpace("").getRotation().getZ());
    // logCurrLLRotationY.append(LimelightHelpers.getBotPose3d_TargetSpace("").get);

    logCurrRobotRotationX.append(drivetrain.getRotation3d().toRotation2d().getDegrees());
    // logCurrLLRotationY.append(drivetrain.getRotation3d().getY());
    // logCurrLLRotationZ.append(drivetrain.getRotation3d().getZ());

    // Setup calculated values.
    // double calcX = 0;
    // double calcY = rawY;
    // double calcR = rawR;

    // Modify calulated values through PID controllers.

    // Apply calculated velocities via the swerve request.
    drivetrain.setControl(
        drive.withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(rawR));
    isFinished = false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
