package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.cmds.elevator.ZeroElevatorEncoderCmdSeq;
import frc.robot.subsystems.Candle.CandleState;
import frc.robot.subsystems.Candle.Color;

public class Elevator extends PARTsSubsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private PeriodicIO mPeriodicIO;
  private Candle candle;

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

  public Elevator(Candle candle) {
    super("Elevator");

    this.candle = candle;
    mPeriodicIO = new PeriodicIO();

    lowerLimitSwitch = new DigitalInput(Constants.Elevator.L_SWITCH_PORT);

    upperLimitLaserCAN = new LaserCan(Constants.Elevator.laserCanId);
    try {
      upperLimitLaserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
      upperLimitLaserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(4, 4, 4, 4));
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

    mElevatorPIDController.setTolerance(Constants.Elevator.kTolerance);

    // Elevator Feedforward
    mElevatorFeedForward = new ElevatorFeedforward(
        Constants.Elevator.kS,
        Constants.Elevator.kG,
        Constants.Elevator.kV,
        Constants.Elevator.kA);

    new Trigger(this::getBottomLimit).onTrue(new ZeroElevatorEncoderCmdSeq(this));
  }

  public enum ElevatorState {
    NONE(0),
    STOW(0),
    L2(Constants.Elevator.L2Height),
    L3(Constants.Elevator.L3Height),
    L4(Constants.Elevator.L4Height),
    A1(Constants.Elevator.LowAlgaeHeight),
    A2(Constants.Elevator.HighAlgaeHeight);

    double height;

    ElevatorState(double i) {
      height = i;
    }
  }

  private static class PeriodicIO {
    double elevator_target = 0.0;
    double elevator_power = 0.0;
    LaserCan.Measurement elevator_measurement = null;

    boolean is_elevator_pos_control = false;
    boolean error = false;
    boolean gantry_blocked = false;

    ElevatorState state = ElevatorState.STOW;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    // top and bottom limit triggered at same time, this is impossible
    if (!mPeriodicIO.error)
      mPeriodicIO.error = getBottomLimit() && getTopLimit();

    mPeriodicIO.elevator_measurement = upperLimitLaserCAN.getMeasurement();

    if (!mPeriodicIO.error) {
      if (mPeriodicIO.is_elevator_pos_control && !mPeriodicIO.gantry_blocked) {
        mElevatorPIDController.setGoal(mPeriodicIO.elevator_target);
        double pidCalc = mElevatorPIDController.atGoal() ? 0
            : mElevatorPIDController.calculate(getElevatorPosition(), mPeriodicIO.elevator_target);
        double ffCalc = mElevatorFeedForward.calculate(mElevatorPIDController.getSetpoint().velocity);

        mPeriodicIO.elevator_power = pidCalc + ffCalc;

        setVoltage(mPeriodicIO.elevator_power);
      } else if (Math.abs(mPeriodicIO.elevator_power) > 0 && !mPeriodicIO.gantry_blocked)
        setSpeed(mPeriodicIO.elevator_power);
      else
        setVoltage(mElevatorFeedForward.calculate(0));
    } else {
      candle.addState(CandleState.ERROR);
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

    if (mPeriodicIO.elevator_measurement != null) {
      super.partsNT.setDouble("Laser/distance", mPeriodicIO.elevator_measurement.distance_mm);
      super.partsNT.setDouble("Laser/ambient", mPeriodicIO.elevator_measurement.ambient);
      super.partsNT.setDouble("Laser/budget_ms", mPeriodicIO.elevator_measurement.budget_ms);
      super.partsNT.setDouble("Laser/status", mPeriodicIO.elevator_measurement.status);
    }

    super.partsNT.setDouble("RPS", getRPS());
    super.partsNT.setDouble("Power", mPeriodicIO.elevator_power);
    super.partsNT.setString("State", mPeriodicIO.state.toString());
    super.partsNT.setBoolean("Is Position Control", mPeriodicIO.is_elevator_pos_control);
  }

  @Override
  public void reset() {
    mLeftEncoder.setPosition(0.0);
  }

  @Override
  public void log() {
    // TODO Auto-generated method stub
    //throw new UnsupportedOperationException("Unimplemented method 'log'");
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/
  public void setGantryBlock(boolean b) {
    mPeriodicIO.gantry_blocked = b;
  }

  public double getRPS() {
    return mLeftEncoder.getVelocity() * 60 / Constants.Elevator.gearRatio; // 16 is the gear reduction
  }

  public ElevatorState getState() {
    return mPeriodicIO.state;
  }

  public void setElevatorPower(double power) {
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = power;
  }

  public Command joystickElevatorControl(CommandXboxController controller) {
    return this.run(() -> {
      double speed = -controller.getRightY() * Constants.Elevator.maxSpeed;
      setElevatorPower(speed);
    }).until(() -> Math.abs(controller.getRightY()) < 0.1).andThen(() -> setElevatorPower(0));
  }

  public Command goToElevatorStow() {
    return this.runOnce(() -> {
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = Constants.Elevator.StowHeight;
      mPeriodicIO.state = ElevatorState.STOW;
    });
  }

  public Command goToElevatorL2() {
    return this.runOnce(() -> {
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = Constants.Elevator.L2Height;
      mPeriodicIO.state = ElevatorState.L2;
    });
  }

  public Command goToElevatorL3() {
    return this.runOnce(() -> {
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = Constants.Elevator.L3Height;
      mPeriodicIO.state = ElevatorState.L3;
    });
  }

  public Command goToElevatorL4() {
    return this.runOnce(() -> {
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = Constants.Elevator.L4Height;
      mPeriodicIO.state = ElevatorState.L4;
    });
  }

  public Command goToAlgaeLow() {
    return this.runOnce(() -> {
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = Constants.Elevator.LowAlgaeHeight;
      mPeriodicIO.state = ElevatorState.A1;
    });
  }

  public Command goToAlgaeHigh() {
    return this.runOnce(() -> {
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = Constants.Elevator.HighAlgaeHeight;
      mPeriodicIO.state = ElevatorState.A2;
    });
  }

  /*
  public Command interrupt() {
    return this.runOnce(() -> {
      mPeriodicIO.is_elevator_pos_control = false;
    });
  }*/

  public boolean isPositionControl() {
    return mPeriodicIO.is_elevator_pos_control;
  }

  public boolean isNotPositionControl() {
    return !mPeriodicIO.is_elevator_pos_control;
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
    //return mPeriodicIO.elevator_measurement != null ? mPeriodicIO.elevator_measurement.distance_mm <= 40 : false;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/

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

  private void setVoltage(double voltage) {
    // Full control in limits
    if (!getBottomLimit() && !getTopLimit())
      mLeftMotor.setVoltage(voltage);
    // Directional control at limits
    else if ((getBottomLimit() && voltage > 0) || (getTopLimit() && voltage < 0))
      mLeftMotor.setVoltage(voltage);
    else
      stop();
  }

}