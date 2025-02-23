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
import frc.robot.Constants;
import frc.robot.subsystems.Candle.CandleState;

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
  }

  public enum ElevatorState {
    ERROR,
    NONE,
    STOW,
    L2,
    L3,
    L4,
    A1,
    A2
  }

  private static class PeriodicIO {
    double elevator_previous_position = 0.0;
    int elevator_position_debounce = 0;

    double elevator_target = 0.0;
    double elevator_power = 0.0;
    LaserCan.Measurement elevator_measurement = null;

    boolean is_elevator_pos_control = false;
    boolean error = false;
    boolean gantry_blocked = false;

    ElevatorState state = ElevatorState.STOW;

    boolean useLaserCan = false;

    boolean elevator_bottom_limit_error = false;
    int elevator_bottom_limit_debounce = 0;
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    //TODO: TEST
    errorTasks();

    mPeriodicIO.elevator_measurement = upperLimitLaserCAN.getMeasurement();

    if (!mPeriodicIO.error) {
      if (mPeriodicIO.is_elevator_pos_control && !mPeriodicIO.gantry_blocked) {
        mElevatorPIDController.setGoal(mPeriodicIO.elevator_target);
        double pidCalc = mElevatorPIDController.atGoal() ? 0
            : mElevatorPIDController.calculate(getElevatorPosition(), mPeriodicIO.elevator_target);
        double ffCalc = mElevatorFeedForward.calculate(mElevatorPIDController.getSetpoint().velocity);

        mPeriodicIO.elevator_power = pidCalc + ffCalc;

        setVoltage(mPeriodicIO.elevator_power);

        mPeriodicIO.elevator_position_debounce++;

        // Check to make sure we move, or trigger error
        if (mPeriodicIO.elevator_position_debounce > 10) {
          mPeriodicIO.elevator_position_debounce = 0;
          if (getElevatorPosition() - mPeriodicIO.elevator_previous_position == 0) {
            mPeriodicIO.error = true;
            setSpeed(0);
          }
          mPeriodicIO.elevator_previous_position = getElevatorPosition();
        }

      } else if (Math.abs(mPeriodicIO.elevator_power) > 0 && !mPeriodicIO.gantry_blocked)
        setSpeed(mPeriodicIO.elevator_power);
      else
        setVoltage(mElevatorFeedForward.calculate(0));

      //reset encoders, only do if lower than 30 to keep coral falls from triggering.
      if (getBottomLimit() && getElevatorPosition() <= Constants.Elevator.bottomLimitPositionErrorMargin)
        resetEncoder();
    }
    // Error controls
    else {
      if (Math.abs(mPeriodicIO.elevator_power) > 0 && !mPeriodicIO.gantry_blocked)
        setSpeed(mPeriodicIO.elevator_power);
      else
        setVoltage(mElevatorFeedForward.calculate(0));
    }
  }

  @Override
  public void stop() {
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = 0.0;

    setSpeedWithoutLimits(0.0);
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
    resetEncoder();
  }

  @Override
  public void log() {
    // TODO Auto-generated method stub
    //throw new UnsupportedOperationException("Unimplemented method 'log'");
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/
  public double getElevatorPosition() {
    return mLeftEncoder.getPosition();
  }

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

  public Command zeroElevatorCommand() {
    return this.run(() -> setSpeedWithoutLimits(Constants.Elevator.homingSpeed))
        .unless(() -> mPeriodicIO.gantry_blocked).until(this::getBottomLimit)
        .andThen(() -> stop());
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

  public boolean getBottomLimit() {
    if (!lowerLimitSwitch.get()) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getTopLimit() {
    return mPeriodicIO.useLaserCan && mPeriodicIO.elevator_measurement != null
        ? mPeriodicIO.elevator_measurement.distance_mm <= Constants.Elevator.maxLaserCanHeight
        : getElevatorPosition() >= Constants.Elevator.maxHeight;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/

  private void setSpeed(double speed) {
    // Full control in limits
    if (!getBottomLimit() && !getTopLimit() && !mPeriodicIO.gantry_blocked)
      setSpeedWithoutLimits(speed);
    // Directional control at limits
    else if (((getBottomLimit() && speed > 0) || (getTopLimit() && speed < 0)) && !mPeriodicIO.gantry_blocked)
      setSpeedWithoutLimits(speed);
    else
      stop();
  }

  private void setSpeedWithoutLimits(double speed) {
    mLeftMotor.set(speed);
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

  private void resetEncoder() {
    mLeftEncoder.setPosition(0.0);
  }

  private void errorTasks() {
    /* Error Conditions 
     * Bottom and top limit hit at same time
     * Laser can in use and the measurement is null or status is not good
     * The bottom limit is hit for more than 10 loop runs and we are reporting a current position higher than the margin of error
    */

    mPeriodicIO.elevator_bottom_limit_error = (getBottomLimit()
        && getElevatorPosition() > Constants.Elevator.bottomLimitPositionErrorMargin);
    if (mPeriodicIO.elevator_bottom_limit_error)
      mPeriodicIO.elevator_bottom_limit_debounce++;
    else
      mPeriodicIO.elevator_bottom_limit_debounce = 0;

    if ((getBottomLimit() && getTopLimit()) ||
        (mPeriodicIO.useLaserCan
            && (mPeriodicIO.elevator_measurement == null || mPeriodicIO.elevator_measurement.status != 0))
        || (mPeriodicIO.elevator_bottom_limit_error && mPeriodicIO.elevator_bottom_limit_debounce >= 10)) {
      // If there wasn't an error report it.
      if (!mPeriodicIO.error) {
        mPeriodicIO.error = true;

        setElevatorPower(0);
        mPeriodicIO.state = ElevatorState.ERROR;
        candle.addState(CandleState.ELEVATOR_ERROR);
      }
    } else {
      // If there was an error remove it
      if (mPeriodicIO.error) {
        mPeriodicIO.error = false;
        mPeriodicIO.state = ElevatorState.NONE;
        candle.removeState(CandleState.ELEVATOR_ERROR);
        zeroElevatorCommand().schedule();
      }
    }
  }
}