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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Candle.CandleState;
import frc.robot.util.PARTs.PARTsCommandController;
import frc.robot.util.PARTs.PARTsSubsystem;

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

  public enum ElevatorState {
    SENSOR_ERROR(-1),
    POS_CTL_TRAVEL_ERROR(-1),
    STOW(ElevatorConstants.StowHeight),
    L2(ElevatorConstants.L2Height),
    L3(ElevatorConstants.L3Height),
    L4(ElevatorConstants.L4Height),
    A1(ElevatorConstants.LowAlgaeHeight),
    A2(ElevatorConstants.HighAlgaeHeight);

    double height;

    ElevatorState(double height) {
      this.height = height;
    }
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

    boolean useLaserCan = true;
    int lasercan_error_debounce = 0;

    boolean elevator_bottom_limit_error = false;
    int elevator_bottom_limit_debounce = 0;
  }

  public Elevator(Candle candle) {
    super("Elevator");

    this.candle = candle;
    mPeriodicIO = new PeriodicIO();

    lowerLimitSwitch = new DigitalInput(ElevatorConstants.L_SWITCH_PORT);

    upperLimitLaserCAN = new LaserCan(ElevatorConstants.laserCanId);
    try {
      upperLimitLaserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
      upperLimitLaserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(4, 4, 4, 4));
      upperLimitLaserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }

    SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    elevatorConfig.smartCurrentLimit(ElevatorConstants.kMaxCurrent);

    elevatorConfig.idleMode(IdleMode.kBrake);
    elevatorConfig.limitSwitch.reverseLimitSwitchEnabled(true);

    // LEFT ELEVATOR MOTOR
    mLeftMotor = new SparkMax(ElevatorConstants.leftElevatorId, MotorType.kBrushless);
    mLeftEncoder = mLeftMotor.getEncoder();
    mLeftMotor.configure(
        elevatorConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // RIGHT ELEVATOR MOTOR
    mRightMotor = new SparkMax(ElevatorConstants.rightElevatorId, MotorType.kBrushless);
    mRightEncoder = mRightMotor.getEncoder();
    mRightMotor.configure(
        elevatorConfig.follow(mLeftMotor, true),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    // Elevator PID
    mElevatorPIDController = new ProfiledPIDController(
        ElevatorConstants.kP,
        ElevatorConstants.kI,
        ElevatorConstants.kD,
        new TrapezoidProfile.Constraints(
            ElevatorConstants.kMaxVelocity,
            ElevatorConstants.kMaxAcceleration));

    mElevatorPIDController.setTolerance(ElevatorConstants.kTolerance);

    // Elevator Feedforward
    mElevatorFeedForward = new ElevatorFeedforward(
        ElevatorConstants.kS,
        ElevatorConstants.kG,
        ElevatorConstants.kV,
        ElevatorConstants.kA);

    new Trigger(this::getBottomLimit)
        .onTrue(new WaitCommand(0.2)
            .andThen(this.runOnce(() -> resetEncoder()))
            .onlyIf(() -> getElevatorPosition() <= ElevatorConstants.bottomLimitPositionErrorMargin));

    super.partsNT.putSmartDashboardSendable("PID", mElevatorPIDController);
    super.partsNT.putSmartDashboardSendable("Zero Elevator", zeroElevatorCommand());
    super.partsNT.putSmartDashboardSendable("Toggle LaserCan Active", toggleLaserCanActive());

    mPeriodicIO.elevator_previous_position = getElevatorPosition();
  }
  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    // TODO: TEST
    errorTasks();

    mPeriodicIO.elevator_measurement = upperLimitLaserCAN.getMeasurement();

    if (!mPeriodicIO.error) {
      if (mPeriodicIO.is_elevator_pos_control && !mPeriodicIO.gantry_blocked) {
        mElevatorPIDController.setGoal(mPeriodicIO.elevator_target);

        if (mPeriodicIO.state == ElevatorState.STOW && !getBottomLimit() && mElevatorPIDController.atGoal()) {
          mPeriodicIO.elevator_power = ElevatorConstants.homingSpeed;
        } else if (mPeriodicIO.state == ElevatorState.STOW && getBottomLimit()) {
          mPeriodicIO.elevator_power = 0;
        } else {
          double pidCalc = mElevatorPIDController.atGoal() ? 0
              : mElevatorPIDController.calculate(getElevatorPosition(), mPeriodicIO.elevator_target);
          double ffCalc = mElevatorFeedForward.calculate(mElevatorPIDController.getSetpoint().velocity);

          mPeriodicIO.elevator_power = pidCalc + ffCalc;
        }
        setVoltage(mPeriodicIO.elevator_power);

        mPeriodicIO.elevator_position_debounce++;

      } else if (Math.abs(mPeriodicIO.elevator_power) > 0 && !mPeriodicIO.gantry_blocked)
        setSpeed(mPeriodicIO.elevator_power);
      else
        setVoltage(mElevatorFeedForward.calculate(0));

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
    super.partsNT.setBoolean("Gantry Blocked", mPeriodicIO.gantry_blocked);
    super.partsNT.setBoolean("Using LaserCan", mPeriodicIO.useLaserCan);
  }

  @Override
  public void reset() {
    resetEncoder();
  }

  @Override
  public void log() {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method 'log'");
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/
  public double getElevatorPosition() {
    return mLeftEncoder.getPosition();
  }

  public void setGantryBlock(boolean b) {
    mPeriodicIO.gantry_blocked = b;
  }

  public double getRPS() {
    return mLeftEncoder.getVelocity() * 60 / ElevatorConstants.gearRatio; // 16 is the gear reduction
  }

  public ElevatorState getState() {
    return mPeriodicIO.state;
  }

  public void setElevatorPower(double power) {
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = power;
  }

  public Command joystickElevatorControl(PARTsCommandController controller) {
    return super.commandFactory("joystickElevatorControl", this.run(() -> {
      double speed = -controller.getRightY() * ElevatorConstants.maxSpeed;
      setElevatorPower(speed);
    }).until(() -> Math.abs(controller.getRightY()) < 0.1).andThen(() -> setElevatorPower(0)));
  }

  public Command elevatorToLevelCommand(ElevatorState state) {
    return super.commandFactory("elevatorToStateCommand", this.runOnce(() -> {
      if (state.height != -1) {
        mPeriodicIO.is_elevator_pos_control = true;
        mPeriodicIO.elevator_target = state.height;
        mPeriodicIO.state = state;

        mElevatorPIDController.reset(getElevatorPosition());
        mElevatorPIDController.setGoal(mPeriodicIO.elevator_target);
      }
    }).andThen(new WaitUntilCommand(() -> mElevatorPIDController.atGoal() || mPeriodicIO.error)));
  }

  public void elevatorToLevel(ElevatorState state) {

    if (state.height != -1) {
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = state.height;
      mPeriodicIO.state = state;
    }

  }

  public Command goToElevatorStow() {
    return super.commandFactory("goToElevatorStow", this.runOnce(() -> {
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = ElevatorConstants.StowHeight;
      mPeriodicIO.state = ElevatorState.STOW;
    }));
  }

  public Command goToElevatorL2() {
    return super.commandFactory("goToElevatorL2", this.runOnce(() -> {
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = ElevatorConstants.L2Height;
      mPeriodicIO.state = ElevatorState.L2;
    }));
  }

  public Command goToElevatorL3() {
    return super.commandFactory("goToElevatorL3", this.runOnce(() -> {
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = ElevatorConstants.L3Height;
      mPeriodicIO.state = ElevatorState.L3;
    }));
  }

  public Command goToElevatorL4() {
    return super.commandFactory("goToElevatorL4", this.runOnce(() -> {
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = ElevatorConstants.L4Height;
      mPeriodicIO.state = ElevatorState.L4;
    }));
  }

  public Command goToAlgaeLow() {
    return super.commandFactory("goToAlgaeLow", this.runOnce(() -> {
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = ElevatorConstants.LowAlgaeHeight;
      mPeriodicIO.state = ElevatorState.A1;
    }));
  }

  public Command goToAlgaeHigh() {
    return super.commandFactory("goToAlgaeHigh", this.runOnce(() -> {
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = ElevatorConstants.HighAlgaeHeight;
      mPeriodicIO.state = ElevatorState.A2;
    }));
  }

  public Command zeroElevatorCommand() {
    return super.commandFactory("zeroElevatorCommand",
        this.run(() -> {
          setSpeedWithoutLimits(ElevatorConstants.homingSpeed);
        })
            .unless(() -> mPeriodicIO.gantry_blocked).until(this::getBottomLimit)
            .andThen(this.runOnce(() -> stop())).andThen(goToElevatorStow()));
  }

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
        ? mPeriodicIO.elevator_measurement.distance_mm <= ElevatorConstants.maxLaserCanHeight
        : getElevatorPosition() >= ElevatorConstants.maxHeight;
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

  private Command toggleLaserCanActive() {
    return super.commandFactory("toggleLaserCanActive",
        this.runOnce(() -> mPeriodicIO.useLaserCan = !mPeriodicIO.useLaserCan));
  }

  private void errorTasks() {
    /*
     * Error Conditions
     * Bottom and top limit hit at same time
     * Laser can in use and the measurement is null or status is not good
     * The bottom limit is hit for more than 10 loop runs and we are reporting a
     * current position higher than the margin of error
     */
    if (mPeriodicIO.elevator_bottom_limit_error)
      mPeriodicIO.elevator_bottom_limit_debounce++;
    else
      mPeriodicIO.elevator_bottom_limit_debounce = 0;

    if ((getBottomLimit() && getTopLimit()) ||
        (mPeriodicIO.useLaserCan
            && (mPeriodicIO.elevator_measurement == null || mPeriodicIO.elevator_measurement.status != 0))
        || (mPeriodicIO.elevator_bottom_limit_error && mPeriodicIO.elevator_bottom_limit_debounce >= 10)) {
      // If there wasn't an error report it.
      mPeriodicIO.lasercan_error_debounce++;

      if (!mPeriodicIO.error) {
        mPeriodicIO.error = true;

        setElevatorPower(0);
        mPeriodicIO.state = ElevatorState.SENSOR_ERROR;
        candle.addState(CandleState.ELEVATOR_ERROR);
      }

      if (mPeriodicIO.lasercan_error_debounce > 10)
        mPeriodicIO.useLaserCan = false;
    } else {
      // If there was an error remove it
      if (mPeriodicIO.error && mPeriodicIO.state != ElevatorState.POS_CTL_TRAVEL_ERROR) {
        mPeriodicIO.error = false;
        mPeriodicIO.state = ElevatorState.STOW;
        candle.removeState(CandleState.ELEVATOR_ERROR);
      }
    }
  }
}