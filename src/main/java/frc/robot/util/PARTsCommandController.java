package frc.robot.util;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.PARTsController.ControllerType;

public class PARTsCommandController {

    private ControllerType controllerType;
    private CommandXboxController xboxController;
    private CommandPS4Controller dualshockController;
    private CommandPS5Controller dualsenseCotroller;
    private CommandJoystick joystick;
    private String err_msg = "";

    public PARTsCommandController(int port) {
        controllerType = ControllerType.XBOX;
        xboxController = new CommandXboxController(port);
        err_msg = "Unimplemented controller button for " + this.controllerType.name();
    }

    public PARTsCommandController(int port, ControllerType controllerType) {
        this.controllerType = controllerType;
        switch (controllerType) {
            case DS4:
                dualshockController = new CommandPS4Controller(port);
                break;
            case DS5:
                dualsenseCotroller = new CommandPS5Controller(port);
                break;
            case OTHER:
                joystick = new CommandJoystick(port);
                break;
            case XBOX:
                xboxController = new CommandXboxController(port);
                break;
            default:
                throw new UnsupportedOperationException("Unknown controller option '" + controllerType + "' for PARTsCommandController.");
        }
        err_msg = "Unimplemented controller button for " + this.controllerType.name();
    }

    /**
     * Constructs a Trigger instance around the A button's digital signal.
     *
     * @return a Trigger instance representing the A button's digital signal
     *         attached
     *         to the {@link CommandScheduler#getDefaultButtonLoop() default
     *         scheduler button loop}.
     * @see #a(EventLoop)
     */
    public Trigger a() {
        return a(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the A button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the A button's digital signal
     *         attached
     *         to the given loop.
     */
    public Trigger a(EventLoop loop) {
        switch (controllerType) {
            case DS4:
                dualshockController.cross();
                break;
            case DS5:
                break;
            case OTHER:
                break;
            case XBOX:
                break;
            default:
                break;
        }
        return button(XboxController.Button.kA.value, loop);
    }

    /**
     * Constructs a Trigger instance around the B button's digital signal.
     *
     * @return a Trigger instance representing the B button's digital signal
     *         attached
     *         to the {@link CommandScheduler#getDefaultButtonLoop() default
     *         scheduler button loop}.
     * @see #b(EventLoop)
     */
    public Trigger b() {
        return b(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the B button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the B button's digital signal
     *         attached
     *         to the given loop.
     */
    public Trigger b(EventLoop loop) {
        return button(XboxController.Button.kB.value, loop);
    }

    /**
     * Constructs a Trigger instance around the X button's digital signal.
     *
     * @return a Trigger instance representing the X button's digital signal
     *         attached
     *         to the {@link CommandScheduler#getDefaultButtonLoop() default
     *         scheduler button loop}.
     * @see #x(EventLoop)
     */
    public Trigger x() {
        return x(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the X button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the X button's digital signal
     *         attached
     *         to the given loop.
     */
    public Trigger x(EventLoop loop) {
        return button(XboxController.Button.kX.value, loop);
    }

    /**
     * Constructs a Trigger instance around the Y button's digital signal.
     *
     * @return a Trigger instance representing the Y button's digital signal
     *         attached
     *         to the {@link CommandScheduler#getDefaultButtonLoop() default
     *         scheduler button loop}.
     * @see #y(EventLoop)
     */
    public Trigger y() {
        return y(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the Y button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the Y button's digital signal
     *         attached
     *         to the given loop.
     */
    public Trigger y(EventLoop loop) {
        return button(XboxController.Button.kY.value, loop);
    }

    /**
     * Constructs a Trigger instance around the left bumper button's digital signal.
     *
     * @return a Trigger instance representing the left bumper button's digital
     *         signal attached
     *         to the {@link CommandScheduler#getDefaultButtonLoop() default
     *         scheduler button loop}.
     * @see #leftBumper(EventLoop)
     */
    public Trigger leftBumper() {
        return leftBumper(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the left bumper button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the left bumper button's digital
     *         signal attached
     *         to the given loop.
     */
    public Trigger leftBumper(EventLoop loop) {
        return button(XboxController.Button.kLeftBumper.value, loop);
    }

    /**
     * Constructs a Trigger instance around the right bumper button's digital
     * signal.
     *
     * @return a Trigger instance representing the right bumper button's digital
     *         signal attached
     *         to the {@link CommandScheduler#getDefaultButtonLoop() default
     *         scheduler button loop}.
     * @see #rightBumper(EventLoop)
     */
    public Trigger rightBumper() {
        return rightBumper(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the right bumper button's digital
     * signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the right bumper button's digital
     *         signal attached
     *         to the given loop.
     */
    public Trigger rightBumper(EventLoop loop) {
        return button(XboxController.Button.kRightBumper.value, loop);
    }

    /**
     * Constructs a Trigger instance around the back button's digital signal.
     *
     * @return a Trigger instance representing the back button's digital signal
     *         attached
     *         to the {@link CommandScheduler#getDefaultButtonLoop() default
     *         scheduler button loop}.
     * @see #back(EventLoop)
     */
    public Trigger back() {
        return back(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the back button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the back button's digital signal
     *         attached
     *         to the given loop.
     */
    public Trigger back(EventLoop loop) {
        return button(XboxController.Button.kBack.value, loop);
    }

    /**
     * Constructs a Trigger instance around the start button's digital signal.
     *
     * @return a Trigger instance representing the start button's digital signal
     *         attached
     *         to the {@link CommandScheduler#getDefaultButtonLoop() default
     *         scheduler button loop}.
     * @see #start(EventLoop)
     */
    public Trigger start() {
        return start(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the start button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the start button's digital signal
     *         attached
     *         to the given loop.
     */
    public Trigger start(EventLoop loop) {
        return button(XboxController.Button.kStart.value, loop);
    }

    /**
     * Constructs a Trigger instance around the left stick button's digital signal.
     *
     * @return a Trigger instance representing the left stick button's digital
     *         signal attached
     *         to the {@link CommandScheduler#getDefaultButtonLoop() default
     *         scheduler button loop}.
     * @see #leftStick(EventLoop)
     */
    public Trigger leftStick() {
        return leftStick(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the left stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the left stick button's digital
     *         signal attached
     *         to the given loop.
     */
    public Trigger leftStick(EventLoop loop) {
        return button(XboxController.Button.kLeftStick.value, loop);
    }

    /**
     * Constructs a Trigger instance around the right stick button's digital signal.
     *
     * @return a Trigger instance representing the right stick button's digital
     *         signal attached
     *         to the {@link CommandScheduler#getDefaultButtonLoop() default
     *         scheduler button loop}.
     * @see #rightStick(EventLoop)
     */
    public Trigger rightStick() {
        return rightStick(CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the right stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return a Trigger instance representing the right stick button's digital
     *         signal attached
     *         to the given loop.
     */
    public Trigger rightStick(EventLoop loop) {
        return button(XboxController.Button.kRightStick.value, loop);
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The
     * returned
     * trigger will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link Trigger} to
     *                  be true. This value
     *                  should be in the range [0, 1] where 0 is the unpressed state
     *                  of the axis.
     * @param loop      the event loop instance to attach the Trigger to.
     * @return a Trigger instance that is true when the left trigger's axis exceeds
     *         the provided
     *         threshold, attached to the given event loop
     */
    public Trigger leftTrigger(double threshold, EventLoop loop) {
        return axisGreaterThan(XboxController.Axis.kLeftTrigger.value, threshold, loop);
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The
     * returned
     * trigger will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link Trigger} to
     *                  be true. This value
     *                  should be in the range [0, 1] where 0 is the unpressed state
     *                  of the axis.
     * @return a Trigger instance that is true when the left trigger's axis exceeds
     *         the provided
     *         threshold, attached to the
     *         {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *         button loop}.
     */
    public Trigger leftTrigger(double threshold) {
        return leftTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The
     * returned trigger
     * will be true when the axis value is greater than 0.5.
     *
     * @return a Trigger instance that is true when the left trigger's axis exceeds
     *         0.5, attached to
     *         the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *         button loop}.
     */
    public Trigger leftTrigger() {
        return leftTrigger(0.5);
    }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger. The
     * returned
     * trigger will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link Trigger} to
     *                  be true. This value
     *                  should be in the range [0, 1] where 0 is the unpressed state
     *                  of the axis.
     * @param loop      the event loop instance to attach the Trigger to.
     * @return a Trigger instance that is true when the right trigger's axis exceeds
     *         the provided
     *         threshold, attached to the given event loop
     */
    public Trigger rightTrigger(double threshold, EventLoop loop) {
        return axisGreaterThan(XboxController.Axis.kRightTrigger.value, threshold, loop);
    }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger. The
     * returned
     * trigger will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link Trigger} to
     *                  be true. This value
     *                  should be in the range [0, 1] where 0 is the unpressed state
     *                  of the axis.
     * @return a Trigger instance that is true when the right trigger's axis exceeds
     *         the provided
     *         threshold, attached to the
     *         {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *         button loop}.
     */
    public Trigger rightTrigger(double threshold) {
        return rightTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger. The
     * returned trigger
     * will be true when the axis value is greater than 0.5.
     *
     * @return a Trigger instance that is true when the right trigger's axis exceeds
     *         0.5, attached to
     *         the {@link CommandScheduler#getDefaultButtonLoop() default scheduler
     *         button loop}.
     */
    public Trigger rightTrigger() {
        return rightTrigger(0.5);
    }

    /**
     * Get the X axis value of left side of the controller. Right is positive.
     *
     * @return The axis value.
     */
    public double getLeftX() {
        return m_hid.getLeftX();
    }

    /**
     * Get the X axis value of right side of the controller. Right is positive.
     *
     * @return The axis value.
     */
    public double getRightX() {
        return m_hid.getRightX();
    }

    /**
     * Get the Y axis value of left side of the controller. Back is positive.
     *
     * @return The axis value.
     */
    public double getLeftY() {
        return m_hid.getLeftY();
    }

    /**
     * Get the Y axis value of right side of the controller. Back is positive.
     *
     * @return The axis value.
     */
    public double getRightY() {
        return m_hid.getRightY();
    }

    /**
     * Get the left trigger axis value of the controller. Note that this axis is
     * bound to the
     * range of [0, 1] as opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    public double getLeftTriggerAxis() {
        return m_hid.getLeftTriggerAxis();
    }

    /**
     * Get the right trigger axis value of the controller. Note that this axis is
     * bound to the
     * range of [0, 1] as opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    public double getRightTriggerAxis() {
        return m_hid.getRightTriggerAxis();
    }
}
