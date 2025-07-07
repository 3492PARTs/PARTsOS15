package frc.robot.subsystems.Elevator;

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
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Candle.CandleState;
import frc.robot.util.PARTs.PARTsCommandController;
import frc.robot.util.PARTs.PARTsCommandUtils;
import frc.robot.util.PARTs.Abstracts.PARTsSubsystem;

public abstract class Elevator extends PARTsSubsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  protected PeriodicIO mPeriodicIO;
  private Candle candle;

  private final ProfiledPIDController mElevatorPIDController;
  protected ElevatorFeedforward mElevatorFeedForward;

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

  protected static class PeriodicIO {
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
    super.partsNT.putSmartDashboardSendable("Zero Elevator", commandZero());
    super.partsNT.putSmartDashboardSendable("Toggle LaserCan Active", toggleLaserCanActive());

    mPeriodicIO.elevator_previous_position = getElevatorPosition();
  }
  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    errorTasks();

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
    super.partsNT.putDouble("Position/Current", getElevatorPosition());
    super.partsNT.putDouble("Position/Target", mPeriodicIO.elevator_target);
    super.partsNT.putBoolean("Position/At Goal", mElevatorPIDController.atGoal());

    super.partsNT.putDouble("Velocity/Current", getRPS());
    super.partsNT.putDouble("Velocity/Setpoint", mElevatorPIDController.getSetpoint().velocity);

    super.partsNT.putBoolean("Limit/Bottom", getBottomLimit());
    super.partsNT.putBoolean("Limit/Top", getTopLimit());

    if (mPeriodicIO.elevator_measurement != null) {
      super.partsNT.putDouble("Laser/distance", mPeriodicIO.elevator_measurement.distance_mm);
      super.partsNT.putDouble("Laser/ambient", mPeriodicIO.elevator_measurement.ambient);
      super.partsNT.putDouble("Laser/budget_ms", mPeriodicIO.elevator_measurement.budget_ms);
      super.partsNT.putDouble("Laser/status", mPeriodicIO.elevator_measurement.status);
    }

    super.partsNT.putDouble("RPS", getRPS());
    super.partsNT.putDouble("Power", mPeriodicIO.elevator_power);
    super.partsNT.putString("State", mPeriodicIO.state.toString());
    super.partsNT.putBoolean("Is Position Control", mPeriodicIO.is_elevator_pos_control);
    super.partsNT.putBoolean("Gantry Blocked", mPeriodicIO.gantry_blocked);
    super.partsNT.putBoolean("Using LaserCan", mPeriodicIO.useLaserCan);
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
  public abstract double getElevatorPosition();

  public void setGantryBlock(boolean b) {
    mPeriodicIO.gantry_blocked = b;
  }

  public abstract double getRPS();

  public ElevatorState getState() {
    return mPeriodicIO.state;
  }

  public void setElevatorPower(double power) {
    mPeriodicIO.is_elevator_pos_control = false;
    mPeriodicIO.elevator_power = power;
  }

  public Command commandJoystickControl(PARTsCommandController controller) {
    return PARTsCommandUtils.setCommandName("commandJoystickControl", this.run(() -> {
      double speed = -controller.getRightY() * ElevatorConstants.maxSpeed;
      setElevatorPower(speed);
    }).until(() -> Math.abs(controller.getRightY()) < 0.1).andThen(() -> setElevatorPower(0)));
  }

  public Command commandToLevel(ElevatorState state) {
    return PARTsCommandUtils.setCommandName("elevatorToStateCommand", this.runOnce(() -> {
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

  public Command commandStow() {
    return PARTsCommandUtils.setCommandName("commandStow", this.runOnce(() -> {
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = ElevatorConstants.StowHeight;
      mPeriodicIO.state = ElevatorState.STOW;
    }));
  }

  public Command commandL2() {
    return PARTsCommandUtils.setCommandName("commandL2", this.runOnce(() -> {
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = ElevatorConstants.L2Height;
      mPeriodicIO.state = ElevatorState.L2;
    }));
  }

  public Command commandL3() {
    return PARTsCommandUtils.setCommandName("commandL3", this.runOnce(() -> {
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = ElevatorConstants.L3Height;
      mPeriodicIO.state = ElevatorState.L3;
    }));
  }

  public Command commandL4() {
    return PARTsCommandUtils.setCommandName("commandL4", this.runOnce(() -> {
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = ElevatorConstants.L4Height;
      mPeriodicIO.state = ElevatorState.L4;
    }));
  }

  public Command commandAlgaeLow() {
    return PARTsCommandUtils.setCommandName("commandAlgaeLow", this.runOnce(() -> {
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = ElevatorConstants.LowAlgaeHeight;
      mPeriodicIO.state = ElevatorState.A1;
    }));
  }

  public Command commandAlgaeHigh() {
    return PARTsCommandUtils.setCommandName("commandAlgaeHigh", this.runOnce(() -> {
      mPeriodicIO.is_elevator_pos_control = true;
      mPeriodicIO.elevator_target = ElevatorConstants.HighAlgaeHeight;
      mPeriodicIO.state = ElevatorState.A2;
    }));
  }

  public Command commandZero() {
    return PARTsCommandUtils.setCommandName("commandZero",
        this.run(() -> {
          setSpeedWithoutLimits(ElevatorConstants.homingSpeed);
        })
            .unless(() -> mPeriodicIO.gantry_blocked).until(this::getBottomLimit)
            .andThen(this.runOnce(() -> stop())).andThen(commandStow()));
  }

  public boolean isPositionControl() {
    return mPeriodicIO.is_elevator_pos_control;
  }

  public abstract boolean getBottomLimit();

  public abstract boolean getTopLimit();

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

  protected abstract void setSpeedWithoutLimits(double speed);

  private void setVoltage(double voltage) {
    // Full control in limits
    if (!getBottomLimit() && !getTopLimit())
      setSpeedVoltageLimits(voltage);
    // Directional control at limits
    else if ((getBottomLimit() && voltage > 0) || (getTopLimit() && voltage < 0))
      setSpeedVoltageLimits(voltage);
    else
      stop();
  }

  protected abstract void setSpeedVoltageLimits(double voltage);

  protected abstract void resetEncoder();

  private Command toggleLaserCanActive() {
    return PARTsCommandUtils.setCommandName("toggleLaserCanActive",
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