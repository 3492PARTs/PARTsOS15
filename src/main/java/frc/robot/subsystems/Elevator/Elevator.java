package frc.robot.subsystems.Elevator;

import java.util.function.Supplier;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ElevatorConstants;
import frc.robot.states.ElevatorState;
import frc.robot.util.PARTs.Classes.PARTsCommandController;
import frc.robot.util.PARTs.Classes.PARTsCommandUtils;
import frc.robot.util.PARTs.Classes.Abstracts.PARTsSubsystem;

public abstract class Elevator extends PARTsSubsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private ElevatorState elevatorState;
  private boolean gantryBlocked = false;

  protected LaserCan.Measurement carriageLaserCan = null;

  private final ProfiledPIDController mElevatorPIDController;
  protected ElevatorFeedforward mElevatorFeedForward;

  /*
   * protected static class PeriodicIO {
   * double elevator_previous_position = 0.0;
   * int elevator_position_debounce = 0;
   * 
   * double elevator_target = 0.0;
   * double elevator_power = 0.0;
   * LaserCan.Measurement elevator_measurement = null;
   * 
   * boolean is_elevator_pos_control = false;
   * boolean error = false;
   * boolean gantry_blocked = false;
   * 
   * ElevatorState state = ElevatorState.STOW;
   * 
   * boolean useLaserCan = true;
   * int lasercan_error_debounce = 0;
   * 
   * boolean elevator_bottom_limit_error = false;
   * int elevator_bottom_limit_debounce = 0;
   * }
   */

  public Elevator() {
    super("Elevator");

    elevatorState = ElevatorState.STOW;

    // Elevator PID
    mElevatorPIDController = new ProfiledPIDController(
        ElevatorConstants.P,
        ElevatorConstants.I,
        ElevatorConstants.D,
        new TrapezoidProfile.Constraints(
            ElevatorConstants.MAX_VELOCITY,
            ElevatorConstants.MAX_ACCELERATION));

    mElevatorPIDController.setTolerance(ElevatorConstants.TOLERANCE);

    // Elevator Feedforward
    mElevatorFeedForward = new ElevatorFeedforward(
        ElevatorConstants.S,
        ElevatorConstants.G,
        ElevatorConstants.V,
        ElevatorConstants.A);

    new Trigger(this::getBottomLimit)
        .onTrue(new WaitCommand(0.2)
            .andThen(this.runOnce(() -> resetEncoder()))
            .onlyIf(() -> getElevatorPosition() <= ElevatorConstants.BOTTOM_LIMIT_POSITION_ERROR_MARGIN));

    super.partsNT.putSmartDashboardSendable("PID", mElevatorPIDController);
    super.partsNT.putSmartDashboardSendable("Zero Elevator", commandZero());
    // super.partsNT.putSmartDashboardSendable("Toggle LaserCan Active",
    // toggleLaserCanActive());
    super.partsNT.putSmartDashboardSendable("Position Commands/STOW", commandStow());
    super.partsNT.putSmartDashboardSendable("Position Commands/L2", commandL2());
    super.partsNT.putSmartDashboardSendable("Position Commands/L3", commandL3());
    super.partsNT.putSmartDashboardSendable("Position Commands/L4", commandL4());
  }
  /*-------------------------------- Generic Subsystem Functions --------------------------------*/

  @Override
  public void periodic() {
    // errorTasks();
    if (gantryBlocked)
      setVoltage(0);
    else {
      if (elevatorState == ElevatorState.STOP) {
        setSpeed(0);
      } else if (elevatorState == ElevatorState.MANUAL)
        setSpeed(elevatorState.getPower());
      else {
        double voltage = 0;

        mElevatorPIDController.setGoal(elevatorState.getTarget());

        if (elevatorState == ElevatorState.STOW && !getBottomLimit() && mElevatorPIDController.atGoal()) {
          voltage = ElevatorConstants.HOMING_SPEED;
        } else if (elevatorState == ElevatorState.STOW && getBottomLimit()) {
          voltage = 0;
        } else {
          double pidCalc = mElevatorPIDController.atGoal() ? 0
              : mElevatorPIDController.calculate(getElevatorPosition(), elevatorState.getTarget());
          double ffCalc = mElevatorFeedForward.calculate(mElevatorPIDController.getSetpoint().velocity);

          voltage = pidCalc + ffCalc;
        }
        setVoltage(voltage);

      }
    }
  }

  @Override
  public void stop() {
    elevatorState = ElevatorState.STOP;
  }

  @Override
  public void outputTelemetry() {
    super.partsNT.putDouble("Position/Current", getElevatorPosition());
    super.partsNT.putDouble("Position/Target", elevatorState.getTarget());
    super.partsNT.putBoolean("Position/At Goal", mElevatorPIDController.atGoal());

    super.partsNT.putDouble("Velocity/Current", getRPS());
    super.partsNT.putDouble("Velocity/Setpoint", mElevatorPIDController.getSetpoint().velocity);

    super.partsNT.putBoolean("Limit/Bottom", getBottomLimit());
    super.partsNT.putBoolean("Limit/Top", getTopLimit());

    if (carriageLaserCan != null) {
      super.partsNT.putDouble("Laser/distance", carriageLaserCan.distance_mm);
      super.partsNT.putDouble("Laser/ambient", carriageLaserCan.ambient);
      super.partsNT.putDouble("Laser/budget_ms", carriageLaserCan.budget_ms);
      super.partsNT.putDouble("Laser/status", carriageLaserCan.status);
    }

    super.partsNT.putDouble("RPS", getRPS());
    super.partsNT.putDouble("Power", elevatorState.getPower());
    super.partsNT.putString("State", elevatorState.toString());
    super.partsNT.putBoolean("Gantry Blocked", gantryBlocked);
    super.partsNT.putBoolean("Using LaserCan", false);
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

  public void setGantryBlocked(boolean b) {
    gantryBlocked = b;
  }

  public abstract double getRPS();

  public ElevatorState getState() {
    return elevatorState;
  }

  public Supplier<ElevatorState> getStateSupplier() {
    return this::getState;
  }

  public void setElevatorPower(double power) {
    elevatorState = ElevatorState.MANUAL;
    try {
      elevatorState.setPower(power);
    } catch (Exception e) {
      // exception is for setting on non-manual states. this can only be manual
    }
  }

  public Command commandJoystickControl(PARTsCommandController controller) {
    return PARTsCommandUtils.setCommandName("commandJoystickControl", this.run(() -> {
      double speed = -controller.getRightY() * ElevatorConstants.MAX_SPEED;
      setElevatorPower(speed);
    }).until(() -> Math.abs(controller.getRightY()) < 0.1).andThen(() -> setElevatorPower(0)));
  }

  public Command commandToLevel(ElevatorState state) {
    return PARTsCommandUtils.setCommandName("elevatorToStateCommand", this.runOnce(() -> {
      toLevel(state);
    }).andThen(new WaitUntilCommand(() -> mElevatorPIDController.atGoal() || !elevatorState.hasTarget())));
  }

  public void toLevel(ElevatorState state) {
    if (state.hasTarget()) {
      elevatorState = state;

      mElevatorPIDController.reset(getElevatorPosition());
      mElevatorPIDController.setGoal(elevatorState.getTarget());
    }
  }

  public Command commandStow() {
    return PARTsCommandUtils.setCommandName("commandStow", this.runOnce(() -> {
      elevatorState = ElevatorState.STOW;
    }));
  }

  public Command commandL2() {
    return PARTsCommandUtils.setCommandName("commandL2", this.runOnce(() -> {
      elevatorState = ElevatorState.L2;
    }));
  }

  public Command commandL3() {
    return PARTsCommandUtils.setCommandName("commandL3", this.runOnce(() -> {
      elevatorState = ElevatorState.L3;
    }));
  }

  public Command commandL4() {
    return PARTsCommandUtils.setCommandName("commandL4", this.runOnce(() -> {
      elevatorState = ElevatorState.L4;
    }));
  }

  public Command commandAlgaeLow() {
    return PARTsCommandUtils.setCommandName("commandAlgaeLow", this.runOnce(() -> {
      elevatorState = ElevatorState.A1;
    }));
  }

  public Command commandAlgaeHigh() {
    return PARTsCommandUtils.setCommandName("commandAlgaeHigh", this.runOnce(() -> {
      elevatorState = ElevatorState.A2;
    }));
  }

  public Command commandZero() {
    return PARTsCommandUtils.setCommandName("commandZero",
        this.run(() -> {
          setSpeedWithoutLimits(ElevatorConstants.HOMING_SPEED);
        })
            .unless(() -> gantryBlocked).until(this::getBottomLimit)
            .andThen(this.runOnce(() -> stop())).andThen(commandStow()));
  }

  public boolean isPositionControl() {
    return elevatorState == ElevatorState.MANUAL;
  }

  public abstract boolean getBottomLimit();

  public abstract boolean getTopLimit();

  /*---------------------------------- Custom Private Functions ---------------------------------*/

  private void setSpeed(double speed) {
    // Full control in limits
    if (!getBottomLimit() && !getTopLimit() && !gantryBlocked)
      setSpeedWithoutLimits(speed);
    // Directional control at limits
    else if (((getBottomLimit() && speed > 0) || (getTopLimit() && speed < 0))
        && !gantryBlocked)
      setSpeedWithoutLimits(speed);
    else
      setSpeedWithoutLimits(0);
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
      setSpeedVoltageLimits(0);
  }

  protected abstract void setSpeedVoltageLimits(double voltage);

  protected abstract void resetEncoder();

  /*
   * private Command toggleLaserCanActive() {
   * return PARTsCommandUtils.setCommandName("toggleLaserCanActive",
   * this.runOnce(() -> mPeriodicIO.useLaserCan = !mPeriodicIO.useLaserCan));
   * }
   */

  /*
   * private void errorTasks() {
   * /*
   * Error Conditions
   * Bottom and top limit hit at same time
   * Laser can in use and the measurement is null or status is not good
   * The bottom limit is hit for more than 10 loop runs and we are reporting a
   * current position higher than the margin of error
   *
   * if (mPeriodicIO.elevator_bottom_limit_error)
   * mPeriodicIO.elevator_bottom_limit_debounce++;
   * else
   * mPeriodicIO.elevator_bottom_limit_debounce = 0;
   * 
   * if ((getBottomLimit() && getTopLimit()) ||
   * (mPeriodicIO.useLaserCan
   * && (carriageLaserCan == null || carriageLaserCan.status != 0))
   * || (mPeriodicIO.elevator_bottom_limit_error &&
   * mPeriodicIO.elevator_bottom_limit_debounce >= 10)) {
   * // If there wasn't an error report it.
   * mPeriodicIO.lasercan_error_debounce++;
   * 
   * if (!mPeriodicIO.error) {
   * mPeriodicIO.error = true;
   * 
   * setElevatorPower(0);
   * mPeriodicIO.state = ElevatorState.SENSOR_ERROR;
   * candle.addState(CandleState.ELEVATOR_ERROR);
   * }
   * 
   * if (mPeriodicIO.lasercan_error_debounce > 10)
   * mPeriodicIO.useLaserCan = false;
   * } else {
   * // If there was an error remove it
   * if (mPeriodicIO.error && mPeriodicIO.state !=
   * ElevatorState.POS_CTL_TRAVEL_ERROR) {
   * mPeriodicIO.error = false;
   * mPeriodicIO.state = ElevatorState.STOW;
   * candle.removeState(CandleState.ELEVATOR_ERROR);
   * }
   * }
   * }
   */
}