package frc.robot.util.PARTs.Classes;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.constants.RobotConstants;
import frc.robot.util.PARTs.Classes.PARTsController.ControllerType;

public class PARTsCommandController {

    private ControllerType controllerType;
    private CommandXboxController xboxController;
    private CommandPS4Controller dualshockController;
    private CommandPS5Controller dualsenseController;
    private CommandJoystick joystick;
    private String err_msg = "";

    public PARTsCommandController(int port) {
        if (RobotConstants.ALLOW_AUTO_CONTROLLER_DETECTION) {
            if (DriverStation.getJoystickIsXbox(port)) {
                controllerType = ControllerType.XBOX;
            } else if (DriverStation.getJoystickName(port).toLowerCase().contains("dualsense")) {
                controllerType = ControllerType.DS5;
            } else if (DriverStation.getJoystickName(port).toLowerCase().contains("dualshock")) {
                // TODO: This might be the wrong name for the DS4.
                controllerType = ControllerType.DS4;
            } else {
                controllerType = ControllerType.OTHER;
            }
        } else {
            controllerType = ControllerType.XBOX;
        }
        initialize(port);
    }

    public PARTsCommandController(int port, ControllerType controllerType) {
        this.controllerType = controllerType;
        initialize(port);
    }

    private void initialize(int port) {
        switch (controllerType) {
            case DS4:
                dualshockController = new CommandPS4Controller(port);
                break;
            case DS5:
                dualsenseController = new CommandPS5Controller(port);
                break;
            case OTHER:
                joystick = new CommandJoystick(port);
                break;
            case XBOX:
                xboxController = new CommandXboxController(port);
                break;
            default:
                throw new UnsupportedOperationException(
                        "Unknown controller option '" + controllerType + "' for PARTsCommandController.");
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
        Trigger val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.cross(loop);
                break;
            case DS5:
                val = dualsenseController.cross(loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.a(loop);
                break;
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
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
        Trigger val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.circle(loop);
                break;
            case DS5:
                val = dualsenseController.circle(loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.b(loop);
                break;
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
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
        Trigger val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.square(loop);
                break;
            case DS5:
                val = dualsenseController.square(loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.x(loop);
                break;
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
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
        Trigger val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.triangle(loop);
                break;
            case DS5:
                val = dualsenseController.triangle(loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.y(loop);
                break;
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
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
        Trigger val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.L1(loop);
                break;
            case DS5:
                val = dualsenseController.L1(loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.leftBumper(loop);
                break;
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
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
        Trigger val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.R1(loop);
                break;
            case DS5:
                val = dualsenseController.R1(loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.rightBumper(loop);
                break;
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
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
        Trigger val;
        switch (controllerType) {
            case DS4:
                throw new UnsupportedOperationException(err_msg);
            case DS5:
                throw new UnsupportedOperationException(err_msg);
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.back(loop);
                break;
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
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
        Trigger val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.options(loop);
                break;
            case DS5:
                val = dualsenseController.options(loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.start(loop);
                break;
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
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
        Trigger val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.L3(loop);
                break;
            case DS5:
                val = dualsenseController.L3(loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.leftStick(loop);
                break;
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
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
        Trigger val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.R3(loop);
                break;
            case DS5:
                val = dualsenseController.R3(loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.rightStick(loop);
                break;
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
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
        Trigger val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.L2(loop);
                break;
            case DS5:
                val = dualsenseController.L2(loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.leftTrigger(threshold, loop);
                break;
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
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
        Trigger val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.R2(loop);
                break;
            case DS5:
                val = dualsenseController.R2(loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.rightTrigger(threshold, loop);
                break;
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
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
        double val = 0;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getLeftX();
                break;
            case DS5:
                val = dualsenseController.getLeftX();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.getLeftX();
                break;
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Get the X axis value of right side of the controller. Right is positive.
     *
     * @return The axis value.
     */
    public double getRightX() {
        double val = 0;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getRightX();
                break;
            case DS5:
                val = dualsenseController.getRightX();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.getRightX();
                break;
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Get the Y axis value of left side of the controller. Back is positive.
     *
     * @return The axis value.
     */
    public double getLeftY() {
        double val = 0;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getLeftY();
                break;
            case DS5:
                val = dualsenseController.getLeftY();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.getLeftY();
                break;
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Get the Y axis value of right side of the controller. Back is positive.
     *
     * @return The axis value.
     */
    public double getRightY() {
        double val = 0;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getRightY();
                break;
            case DS5:
                val = dualsenseController.getRightY();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.getRightY();
                break;
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Get the left trigger axis value of the controller. Note that this axis is
     * bound to the
     * range of [0, 1] as opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    public double getLeftTriggerAxis() {
        double val = 0;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getL2Axis();
                break;
            case DS5:
                val = dualsenseController.getL2Axis();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.getLeftTriggerAxis();
                break;
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Get the right trigger axis value of the controller. Note that this axis is
     * bound to the
     * range of [0, 1] as opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    public double getRightTriggerAxis() {
        double val = 0;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getR2Axis();
                break;
            case DS5:
                val = dualsenseController.getR2Axis();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.getRightTriggerAxis();
                break;
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Constructs a Trigger instance based around this angle of the default (index
     * 0) POV on the HID,
     * attached to {@link CommandScheduler#getDefaultButtonLoop() the default
     * command scheduler button
     * loop}.
     *
     * <p>
     * The POV angles start at 0 in the up direction, and increase clockwise (e.g.
     * right is 90,
     * upper-left is 315).
     *
     * @param angle POV angle in degrees, or -1 for the center / not pressed.
     * @return a Trigger instance based around this angle of a POV on the HID.
     */
    public Trigger pov(int angle) {
        return pov(0, angle, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance based around this angle of a POV on the HID.
     *
     * <p>
     * The POV angles start at 0 in the up direction, and increase clockwise (e.g.
     * right is 90,
     * upper-left is 315).
     *
     * @param pov   index of the POV to read (starting at 0). Defaults to 0.
     * @param angle POV angle in degrees, or -1 for the center / not pressed.
     * @param loop  the event loop instance to attach the event to. Defaults to
     *              {@link
     *              CommandScheduler#getDefaultButtonLoop() the default command
     *              scheduler button loop}.
     * @return a Trigger instance based around this angle of a POV on the HID.
     */
    public Trigger pov(int pov, int angle, EventLoop loop) {
        Trigger val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.pov(pov, angle, loop);
                break;
            case DS5:
                val = dualsenseController.pov(pov, angle, loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.pov(pov, angle, loop);
                break;
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Constructs a Trigger instance based around the 0 degree angle (up) of the
     * default (index 0) POV
     * on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop() the
     * default command
     * scheduler button loop}.
     *
     * @return a Trigger instance based around the 0 degree angle of a POV on the
     *         HID.
     */
    public Trigger povUp() {
        return pov(0);
    }

    /**
     * Constructs a Trigger instance based around the 45 degree angle (right up) of
     * the default (index
     * 0) POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop()
     * the default
     * command scheduler button loop}.
     *
     * @return a Trigger instance based around the 45 degree angle of a POV on the
     *         HID.
     */
    public Trigger povUpRight() {
        return pov(45);
    }

    /**
     * Constructs a Trigger instance based around the 90 degree angle (right) of the
     * default (index 0)
     * POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop()
     * the default command
     * scheduler button loop}.
     *
     * @return a Trigger instance based around the 90 degree angle of a POV on the
     *         HID.
     */
    public Trigger povRight() {
        return pov(90);
    }

    /**
     * Constructs a Trigger instance based around the 135 degree angle (right down)
     * of the default
     * (index 0) POV on the HID, attached to
     * {@link CommandScheduler#getDefaultButtonLoop() the
     * default command scheduler button loop}.
     *
     * @return a Trigger instance based around the 135 degree angle of a POV on the
     *         HID.
     */
    public Trigger povDownRight() {
        return pov(135);
    }

    /**
     * Constructs a Trigger instance based around the 180 degree angle (down) of the
     * default (index 0)
     * POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop()
     * the default command
     * scheduler button loop}.
     *
     * @return a Trigger instance based around the 180 degree angle of a POV on the
     *         HID.
     */
    public Trigger povDown() {
        return pov(180);
    }

    /**
     * Constructs a Trigger instance based around the 225 degree angle (down left)
     * of the default
     * (index 0) POV on the HID, attached to
     * {@link CommandScheduler#getDefaultButtonLoop() the
     * default command scheduler button loop}.
     *
     * @return a Trigger instance based around the 225 degree angle of a POV on the
     *         HID.
     */
    public Trigger povDownLeft() {
        return pov(225);
    }

    /**
     * Constructs a Trigger instance based around the 270 degree angle (left) of the
     * default (index 0)
     * POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop()
     * the default command
     * scheduler button loop}.
     *
     * @return a Trigger instance based around the 270 degree angle of a POV on the
     *         HID.
     */
    public Trigger povLeft() {
        return pov(270);
    }

    /**
     * Constructs a Trigger instance based around the 315 degree angle (left up) of
     * the default (index
     * 0) POV on the HID, attached to {@link CommandScheduler#getDefaultButtonLoop()
     * the default
     * command scheduler button loop}.
     *
     * @return a Trigger instance based around the 315 degree angle of a POV on the
     *         HID.
     */
    public Trigger povUpLeft() {
        return pov(315);
    }

    /**
     * Constructs a Trigger instance based around the center (not pressed) position
     * of the default
     * (index 0) POV on the HID, attached to
     * {@link CommandScheduler#getDefaultButtonLoop() the
     * default command scheduler button loop}.
     *
     * @return a Trigger instance based around the center position of a POV on the
     *         HID.
     */
    public Trigger povCenter() {
        return pov(-1);
    }

    /**
     * Constructs a Trigger instance that is true when the axis value is less than
     * {@code threshold},
     * attached to {@link CommandScheduler#getDefaultButtonLoop() the default
     * command scheduler button
     * loop}.
     *
     * @param axis      The axis to read, starting at 0
     * @param threshold The value below which this trigger should return true.
     * @return a Trigger instance that is true when the axis value is less than the
     *         provided
     *         threshold.
     */
    public Trigger axisLessThan(int axis, double threshold) {
        return axisLessThan(axis, threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance that is true when the axis value is less than
     * {@code threshold},
     * attached to the given loop.
     *
     * @param axis      The axis to read, starting at 0
     * @param threshold The value below which this trigger should return true.
     * @param loop      the event loop instance to attach the trigger to
     * @return a Trigger instance that is true when the axis value is less than the
     *         provided
     *         threshold.
     */
    public Trigger axisLessThan(int axis, double threshold, EventLoop loop) {
        Trigger val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.axisLessThan(axis, threshold, loop);
                break;
            case DS5:
                val = dualsenseController.axisLessThan(axis, threshold, loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.axisLessThan(axis, threshold, loop);
                break;
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Constructs a Trigger instance that is true when the axis value is less than
     * {@code threshold},
     * attached to {@link CommandScheduler#getDefaultButtonLoop() the default
     * command scheduler button
     * loop}.
     *
     * @param axis      The axis to read, starting at 0
     * @param threshold The value above which this trigger should return true.
     * @return a Trigger instance that is true when the axis value is greater than
     *         the provided
     *         threshold.
     */
    public Trigger axisGreaterThan(int axis, double threshold) {
        return axisGreaterThan(axis, threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance that is true when the axis value is greater
     * than {@code
     * threshold}, attached to the given loop.
     *
     * @param axis      The axis to read, starting at 0
     * @param threshold The value above which this trigger should return true.
     * @param loop      the event loop instance to attach the trigger to.
     * @return a Trigger instance that is true when the axis value is greater than
     *         the provided
     *         threshold.
     */
    public Trigger axisGreaterThan(int axis, double threshold, EventLoop loop) {
        Trigger val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.axisGreaterThan(axis, threshold, loop);
                break;
            case DS5:
                val = dualsenseController.axisGreaterThan(axis, threshold, loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.axisGreaterThan(axis, threshold, loop);
                break;
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Constructs a Trigger instance that is true when the axis magnitude value is
     * greater than {@code
     * threshold}, attached to the given loop.
     *
     * @param axis      The axis to read, starting at 0
     * @param threshold The value above which this trigger should return true.
     * @param loop      the event loop instance to attach the trigger to.
     * @return a Trigger instance that is true when the axis magnitude value is
     *         greater than the
     *         provided threshold.
     */
    public Trigger axisMagnitudeGreaterThan(int axis, double threshold, EventLoop loop) {
        Trigger val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.axisMagnitudeGreaterThan(axis, threshold, loop);
                break;
            case DS5:
                val = dualsenseController.axisMagnitudeGreaterThan(axis, threshold, loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.axisMagnitudeGreaterThan(axis, threshold, loop);
                break;
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Constructs a Trigger instance that is true when the axis magnitude value is
     * greater than {@code
     * threshold}, attached to {@link CommandScheduler#getDefaultButtonLoop() the
     * default command
     * scheduler button loop}.
     *
     * @param axis      The axis to read, starting at 0
     * @param threshold The value above which this trigger should return true.
     * @return a Trigger instance that is true when the deadbanded axis value is
     *         active (non-zero).
     */
    public Trigger axisMagnitudeGreaterThan(int axis, double threshold) {
        return axisMagnitudeGreaterThan(
                axis, threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }
}
