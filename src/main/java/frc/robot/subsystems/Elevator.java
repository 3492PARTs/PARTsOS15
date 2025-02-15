package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends PARTsSubsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private PeriodicIO mPeriodicIO;

  // private static final double kPivotCLRampRate = 0.5;
  // private static final double kCLRampRate = 0.5;

  private SparkMax mLeftMotor;
  private RelativeEncoder mLeftEncoder;
  private RelativeEncoder mRightEncoder;

  private SparkClosedLoopController mLeftPIDController;

  private SparkMax mRightMotor;

  private DigitalInput elevatorLimit;

  private TrapezoidProfile mProfile;
  private TrapezoidProfile.State mCurState = new TrapezoidProfile.State();
  private TrapezoidProfile.State mGoalState = new TrapezoidProfile.State();
  private double prevUpdateTime = Timer.getFPGATimestamp();

  public Elevator() {
    super("Elevator");

    mPeriodicIO = new PeriodicIO();

    elevatorLimit = new DigitalInput(Constants.Elevator.L_SWITCH_PORT);

    SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    elevatorConfig.closedLoop
        .pid(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD)
        .iZone(Constants.Elevator.kIZone);

    elevatorConfig.smartCurrentLimit(Constants.Elevator.kMaxCurrent);

    elevatorConfig.idleMode(IdleMode.kBrake);
    elevatorConfig.limitSwitch.reverseLimitSwitchEnabled(true);
    //elevatorConfig.alternateEncoder.apply(new AlternateEncoderConfig().countsPerRevolution(8192));

    // LEFT ELEVATOR MOTOR
    mLeftMotor = new SparkMax(Constants.Elevator.leftElevatorId, MotorType.kBrushless);
    mLeftEncoder = mLeftMotor.getEncoder();
    mLeftPIDController = mLeftMotor.getClosedLoopController();
    mLeftMotor.configure(
        elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // RIGHT ELEVATOR MOTOR
    mRightMotor = new SparkMax(Constants.Elevator.rightElevatorId, MotorType.kBrushless);
    mRightEncoder = mRightMotor.getEncoder();
    mRightMotor.configure(
        elevatorConfig.follow(mLeftMotor, true),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    mProfile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(
            Constants.Elevator.kMaxVelocity,
            Constants.Elevator.kMaxAcceleration));
  }

  public enum ElevatorState {
    NONE,
    STOW,
    L2,
    L3,
    L4,
    A1,
    A2
  }

  private static class PeriodicIO {
    double elevator_target = 0.0;
    double elevator_power = 0.0;

    boolean is_elevator_pos_control = false;

    ElevatorState state = ElevatorState.STOW;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    super.periodic();
    // TODO: Use this pattern to only drive slowly when we're really high up
    // if(mPivotEncoder.getPosition() > Constants.kPivotScoreCount) {
    // mPeriodicIO.is_pivot_low = true;
    // } else {
    // mPeriodicIO.is_pivot_low = false;
    // }
    double curTime = Timer.getFPGATimestamp();
    double dt = curTime - prevUpdateTime;
    prevUpdateTime = curTime;
    if (mPeriodicIO.is_elevator_pos_control) {
      // Update goal
      mGoalState.position = mPeriodicIO.elevator_target;

      // Calculate new state
      prevUpdateTime = curTime;
      mCurState = mProfile.calculate(dt, mCurState, mGoalState);

      // Set PID controller to new state
      mLeftPIDController.setReference(
          mCurState.position,
          SparkBase.ControlType.kPosition,
          ClosedLoopSlot.kSlot0,
          Constants.Elevator.kG,
          ArbFFUnits.kVoltage);
    } else {
      mCurState.position = mLeftEncoder.getPosition();
      mCurState.velocity = 0;
      mLeftMotor.set(mPeriodicIO.elevator_power);
    }
  }

  public void stop() {
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = 0.0;

    mLeftMotor.set(0.0);
  }

  public void zeroEncoders() {
    mLeftEncoder.setPosition(0);
    mRightEncoder.setPosition(0);
  }

  public boolean getLimitSwitch() {
    if (!elevatorLimit.get()) {
      return true;
    } 
    
    else {
      return false;
    }
  }

  public BooleanSupplier getLimitSwitchSupplier () {
    return this::getLimitSwitch;
  }

  public void setSpeed(double speed) {
    mLeftMotor.set(speed);
   // mRightMotor.set(speed);
  }

  @Override
  public void outputTelemetry() {
    super.partsNT.setDouble("Position/Current", mLeftEncoder.getPosition());

    super.partsNT.setDouble("Position/Target", mPeriodicIO.elevator_target);
    super.partsNT.setDouble("Velocity/Current", mLeftEncoder.getVelocity());

    super.partsNT.setDouble("Position/Setpoint", mCurState.position);
    super.partsNT.setDouble("Velocity/Setpoint", mCurState.velocity);

    super.partsNT.setDouble("Current/Left", mLeftMotor.getOutputCurrent());
    super.partsNT.setDouble("Current/Right", mRightMotor.getOutputCurrent());

    super.partsNT.setDouble("Output/Left", mLeftMotor.getAppliedOutput());
    super.partsNT.setDouble("Output/Right", mRightMotor.getAppliedOutput());

    super.partsNT.setString("State", mPeriodicIO.state.toString());

    super.partsNT.setBoolean("Limit Switch", getLimitSwitch());
  }

//TODO: Remember to swap for absolute
  public void reset() {
    mLeftEncoder.setPosition(0.0);
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public ElevatorState getState() {
    return mPeriodicIO.state;
  }

  public void setElevatorPower(double power) {
    //putNumber("setElevatorPower", power);
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = power;
  }

  public void goToElevatorStow() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.StowHeight;
    mPeriodicIO.state = ElevatorState.STOW;
  }

  public void goToElevatorL2() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.L2Height;
    mPeriodicIO.state = ElevatorState.L2;
  }

  public void goToElevatorL3() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.L3Height;
    mPeriodicIO.state = ElevatorState.L3;
  }

  public void goToElevatorL4() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.L4Height;
    mPeriodicIO.state = ElevatorState.L4;
  }

  public void goToAlgaeLow() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.LowAlgaeHeight;
    mPeriodicIO.state = ElevatorState.A1;
  }

  public void goToAlgaeHigh() {
    mPeriodicIO.is_elevator_pos_control = true;
    mPeriodicIO.elevator_target = Constants.Elevator.HighAlgaeHeight;
    mPeriodicIO.state = ElevatorState.A2;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
}