package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
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

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;

public class Elevator extends PARTsSubsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private PeriodicIO mPeriodicIO;

  // private static final double kPivotCLRampRate = 0.5;
  // private static final double kCLRampRate = 0.5;

  private SparkMax mRightMotor;
  protected SparkMax mLeftMotor;

  protected RelativeEncoder mLeftEncoder;
  private RelativeEncoder mRightEncoder;

  private final ProfiledPIDController mElevatorPIDController;
  private final ElevatorFeedforward mElevatorFeedForward;

  private DigitalInput lowerLimitSwitch;
  private LaserCan upperLimitLaserCAN;

  public Elevator() {
    super("Elevator");

    mPeriodicIO = new PeriodicIO();

    lowerLimitSwitch = new DigitalInput(Constants.Elevator.L_SWITCH_PORT);

    upperLimitLaserCAN = new LaserCan(Constants.Elevator.laserCanId);
    try {
      upperLimitLaserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
      upperLimitLaserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      upperLimitLaserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }

    SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    elevatorConfig.smartCurrentLimit(Constants.Elevator.kMaxCurrent);

    elevatorConfig.idleMode(IdleMode.kBrake);
    elevatorConfig.limitSwitch.reverseLimitSwitchEnabled(true);

    // LEFT ELEVATOR MOTOR
    mLeftMotor = new SparkMax(Constants.Elevator.leftElevatorId, MotorType.kBrushless);
    mLeftEncoder = mLeftMotor.getEncoder();
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

    // Elevator PID
    mElevatorPIDController = new ProfiledPIDController(
        Constants.Elevator.kP,
        Constants.Elevator.kI,
        Constants.Elevator.kD,
        new TrapezoidProfile.Constraints(
            Constants.Elevator.kMaxVelocity,
            Constants.Elevator.kMaxAcceleration));

    // Elevator Feedforward
    mElevatorFeedForward = new ElevatorFeedforward(
        Constants.Elevator.kS,
        Constants.Elevator.kG,
        Constants.Elevator.kV,
        Constants.Elevator.kA);


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
    double elevator_measurement = 0.0;

    boolean is_elevator_pos_control = false;

    ElevatorState state = ElevatorState.STOW;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    mPeriodicIO.elevator_measurement = upperLimitLaserCAN.getMeasurement().distance_mm;
    // TODO: Use this pattern to only drive slowly when we're really high up
    // if(mPivotEncoder.getPosition() > Constants.kPivotScoreCount) {
    // mPeriodicIO.is_pivot_low = true;
    // } else {
    // mPeriodicIO.is_pivot_low = false;
    // }
    if (mPeriodicIO.is_elevator_pos_control) {
      mElevatorPIDController.setGoal(mPeriodicIO.elevator_target);
      double pidCalc = mElevatorPIDController.atGoal() ? 0 : mElevatorPIDController.calculate(getElevatorPosition(), mPeriodicIO.elevator_target);
      //double ffCalc = mElevatorFeedForward.calculate(mElevatorPIDController.getSetpoint().velocity);

      mPeriodicIO.elevator_power = pidCalc;// + ffCalc;

      setSpeed(mPeriodicIO.elevator_power);
    } else {
      setSpeed(mPeriodicIO.elevator_power);
    }
  }

  public double getElevatorPosition() {
    return mLeftEncoder.getPosition();
  }

  @Override
  public void stop() {
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = 0.0;

    mLeftMotor.set(0.0);
  }

  public boolean getBottomLimit() {
    if (!lowerLimitSwitch.get()) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getTopLimit() {
    return getElevatorPosition() >= Constants.Elevator.maxHeight;
  }

  private void setSpeed(double speed) {
    // Full control in limits
    if (!getBottomLimit() && !getTopLimit()) 
      mLeftMotor.set(speed);
    // Directional control at limits
    else if ((getBottomLimit() && speed > 0) || (getTopLimit() && speed < 0)) 
      mLeftMotor.set(speed);
    else
      stop();
  }

  @Override
  public void outputTelemetry() {
    super.partsNT.setDouble("Position/Current", getElevatorPosition());
    super.partsNT.setDouble("Position/Target", mPeriodicIO.elevator_target);
    super.partsNT.setBoolean("Position/At Goal", mElevatorPIDController.atGoal());

    super.partsNT.setDouble("Velocity/Current", getRPS());
    super.partsNT.setDouble("Velocity/Setpoint", mElevatorPIDController.getSetpoint().velocity);

    super.partsNT.setDouble("Current/Left", mLeftMotor.getOutputCurrent());
    super.partsNT.setDouble("Current/Right", mRightMotor.getOutputCurrent());

    super.partsNT.setDouble("Output/Left", mLeftMotor.getAppliedOutput());
    super.partsNT.setDouble("Output/Right", mRightMotor.getAppliedOutput());
    

    super.partsNT.setBoolean("Limit/Bottom", getBottomLimit());
    super.partsNT.setBoolean("Limit/Top", getTopLimit());
    super.partsNT.setDouble("Limit/TopMeasurement", mPeriodicIO.elevator_measurement);

    super.partsNT.setDouble("RPS", getRPS());
    super.partsNT.setDouble("Power", mPeriodicIO.elevator_power);
    super.partsNT.setString("State", mPeriodicIO.state.toString());
  }

  @Override
  public void reset() {
    mLeftEncoder.setPosition(0.0);
  }

  public double getRPS() {
    return mLeftEncoder.getVelocity() * 60 / Constants.Elevator.gearRatio; // 16 is the gear reduction
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/

  public ElevatorState getState() {
    return mPeriodicIO.state;
  }

  public void setElevatorPower(double power) {
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

  @Override
  public void log() {
    // TODO Auto-generated method stub
    //throw new UnsupportedOperationException("Unimplemented method 'log'");
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
}