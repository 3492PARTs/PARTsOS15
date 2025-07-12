package frc.robot.util.PARTs.Classes;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.constants.RobotConstants;

public class PARTsController {

    public static enum ControllerType {
        /**
         * Controller type for Xbox Controls.
         */
        XBOX(),
        /**
         * Controller type for Sony Dualshock 4 controllers.
         */
        DS4(),
        /**
         * Controller type for Sony Dualsense controllers.
         */
        DS5(),
        /**
         * Controller type for other controllers.
         */
        OTHER();
    }

    private ControllerType controllerType;
    private XboxController xboxController;
    private PS4Controller dualshockController;
    private PS5Controller dualsenseController;
    // TODO: Finish joystick by allowing an array of custom button codes that will map to the xbox controller.
    private Joystick joystick;
    private String err_msg = "";

    /**
     * Creates a new PARTsController that defaults to the {@link ControllerType#XBOX} type.
     * @param port The controller port.
     */
    public PARTsController(int port) {
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

    public PARTsController(int port, ControllerType controllerType) {
        this.controllerType = controllerType;
        initialize(port);
    }

    private void initialize(int port) {
        switch (controllerType) {
            case DS4:
                dualshockController = new PS4Controller(port);
                break;
            case DS5:
                dualsenseController = new PS5Controller(port);
                break;
            case XBOX:
                xboxController = new XboxController(port);
                break;
            case OTHER:
                joystick = new Joystick(port);
                break;
            default:
                throw new UnsupportedOperationException(
                        "Unknown controller option '" + controllerType + "' for PARTsController.");
        }
        err_msg = "Unimplemented controller button for " + this.controllerType.name();
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
            case XBOX:
                val = xboxController.getLeftTriggerAxis();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Constructs an event instance around the axis value of the left trigger. The
     * returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link BooleanEvent}
     *                  to be true. This
     *                  value should be in the range [0, 1] where 0 is the unpressed
     *                  state of the axis.
     * @param loop      the event loop instance to attach the event to.
     * @return an event instance that is true when the left trigger's axis exceeds
     *         the provided
     *         threshold, attached to the given event loop
     */
    public BooleanEvent leftTrigger(double threshold, EventLoop loop) {
        BooleanEvent val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.L2(loop);
                break;
            case DS5:
                val = dualsenseController.L2(loop);
                break;
            case XBOX:
                val = xboxController.leftTrigger(threshold, loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Constructs an event instance around the axis value of the left trigger. The
     * returned trigger
     * will be true when the axis value is greater than 0.5.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance that is true when the left trigger's axis exceeds
     *         the provided
     *         threshold, attached to the given event loop
     */
    public BooleanEvent leftTrigger(EventLoop loop) {
        return leftTrigger(0.5, loop);
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
                val = dualshockController.getL2Axis();
                break;
            case DS5:
                val = dualsenseController.getL2Axis();
                break;
            case XBOX:
                val = xboxController.getLeftTriggerAxis();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Constructs an event instance around the axis value of the right trigger. The
     * returned trigger
     * will be true when the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link BooleanEvent}
     *                  to be true. This
     *                  value should be in the range [0, 1] where 0 is the unpressed
     *                  state of the axis.
     * @param loop      the event loop instance to attach the event to.
     * @return an event instance that is true when the right trigger's axis exceeds
     *         the provided
     *         threshold, attached to the given event loop
     */
    public BooleanEvent rightTrigger(double threshold, EventLoop loop) {
        BooleanEvent val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.L2(loop);
                break;
            case DS5:
                val = dualsenseController.L2(loop);
                break;
            case XBOX:
                val = xboxController.leftTrigger(threshold, loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Constructs an event instance around the axis value of the right trigger. The
     * returned trigger
     * will be true when the axis value is greater than 0.5.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance that is true when the right trigger's axis exceeds
     *         the provided
     *         threshold, attached to the given event loop
     */
    public BooleanEvent rightTrigger(EventLoop loop) {
        return rightTrigger(0.5, loop);
    }

    /**
     * Read the value of the A button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getAButton() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getCrossButton();
                break;
            case DS5:
                val = dualsenseController.getCrossButton();
                break;
            case XBOX:
                val = xboxController.getAButton();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Whether the A button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getAButtonPressed() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getCrossButtonPressed();
                break;
            case DS5:
                val = dualsenseController.getCrossButtonPressed();
                break;
            case XBOX:
                val = xboxController.getAButtonPressed();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Whether the A button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getAButtonReleased() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getCrossButtonReleased();
                break;
            case DS5:
                val = dualsenseController.getCrossButtonReleased();
                break;
            case XBOX:
                val = xboxController.getAButtonReleased();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Constructs an event instance around the A button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the A button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent a(EventLoop loop) {
        BooleanEvent val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.cross(loop);
                break;
            case DS5:
                val = dualsenseController.cross(loop);
                break;
            case XBOX:
                val = xboxController.a(loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Read the value of the B button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getBButton() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getCircleButton();
                break;
            case DS5:
                val = dualsenseController.getCircleButton();
                break;
            case XBOX:
                val = xboxController.getBButton();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Whether the B button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getBButtonPressed() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getCircleButtonPressed();
                break;
            case DS5:
                val = dualsenseController.getCircleButtonPressed();
                break;
            case XBOX:
                val = xboxController.getBButtonPressed();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Whether the B button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getBButtonReleased() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getCircleButtonReleased();
                break;
            case DS5:
                val = dualsenseController.getCircleButtonReleased();
                break;
            case XBOX:
                val = xboxController.getBButtonReleased();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Constructs an event instance around the B button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the B button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent b(EventLoop loop) {
        BooleanEvent val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.circle(loop);
                break;
            case DS5:
                val = dualsenseController.circle(loop);
                break;
            case XBOX:
                val = xboxController.b(loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Read the value of the X button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getXButton() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getSquareButton();
                break;
            case DS5:
                val = dualsenseController.getSquareButton();
                break;
            case XBOX:
                val = xboxController.getXButton();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Whether the X button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getXButtonPressed() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getSquareButtonPressed();
                break;
            case DS5:
                val = dualsenseController.getSquareButtonPressed();
                break;
            case XBOX:
                val = xboxController.getXButtonPressed();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Whether the X button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getXButtonReleased() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getSquareButtonReleased();
                break;
            case DS5:
                val = dualsenseController.getSquareButtonReleased();
                break;
            case XBOX:
                val = xboxController.getXButtonReleased();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Constructs an event instance around the X button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the X button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent x(EventLoop loop) {
        BooleanEvent val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.square(loop);
                break;
            case DS5:
                val = dualsenseController.square(loop);
                break;
            case XBOX:
                val = xboxController.x(loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Read the value of the Y button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getYButton() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getTriangleButton();
                break;
            case DS5:
                val = dualsenseController.getTriangleButton();
                break;
            case XBOX:
                val = xboxController.getYButton();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Whether the Y button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getYButtonPressed() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getTriangleButtonPressed();
                break;
            case DS5:
                val = dualsenseController.getTriangleButtonPressed();
                break;
            case XBOX:
                val = xboxController.getYButtonPressed();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Whether the Y button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getYButtonReleased() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getTriangleButtonReleased();
                break;
            case DS5:
                val = dualsenseController.getTriangleButtonReleased();
                break;
            case XBOX:
                val = xboxController.getYButtonReleased();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Constructs an event instance around the Y button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the Y button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent y(EventLoop loop) {
        BooleanEvent val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.triangle(loop);
                break;
            case DS5:
                val = dualsenseController.triangle(loop);
                break;
            case XBOX:
                val = xboxController.y(loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Read the value of the left bumper button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getLeftBumperButton() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getL1Button();
                break;
            case DS5:
                val = dualsenseController.getL1Button();
                break;
            case XBOX:
                val = xboxController.getLeftBumperButton();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Whether the left bumper button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getLeftBumperButtonPressed() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getL1ButtonPressed();
                break;
            case DS5:
                val = dualsenseController.getL1ButtonPressed();
                break;
            case XBOX:
                val = xboxController.getLeftBumperButtonPressed();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Whether the left bumper button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getLeftBumperButtonReleased() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getL1ButtonReleased();
                break;
            case DS5:
                val = dualsenseController.getL1ButtonReleased();
                break;
            case XBOX:
                val = xboxController.getLeftBumperButtonReleased();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Constructs an event instance around the left bumper button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the left bumper button's digital
     *         signal
     *         attached to the given loop.
     */
    public BooleanEvent leftBumper(EventLoop loop) {
        BooleanEvent val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.L1(loop);
                break;
            case DS5:
                val = dualsenseController.L1(loop);
                break;
            case XBOX:
                val = xboxController.leftBumper(loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Read the value of the right bumper button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getRightBumperButton() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getR1Button();
                break;
            case DS5:
                val = dualsenseController.getR1Button();
                break;
            case XBOX:
                val = xboxController.getRightBumperButton();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Whether the right bumper button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getRightBumperButtonPressed() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getR1ButtonPressed();
                break;
            case DS5:
                val = dualsenseController.getR1ButtonPressed();
                break;
            case XBOX:
                val = xboxController.getRightBumperButtonPressed();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Whether the right bumper button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getRightBumperButtonReleased() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getR1ButtonReleased();
                break;
            case DS5:
                val = dualsenseController.getR1ButtonReleased();
                break;
            case XBOX:
                val = xboxController.getRightBumperButtonReleased();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Constructs an event instance around the right bumper button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right bumper button's digital
     *         signal
     *         attached to the given loop.
     */
    public BooleanEvent rightBumper(EventLoop loop) {
        BooleanEvent val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.R1(loop);
                break;
            case DS5:
                val = dualsenseController.R1(loop);
                break;
            case XBOX:
                val = xboxController.rightBumper(loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Read the value of the back button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getBackButton() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                //val = dualshockController.getR1Button();
                throw new UnsupportedOperationException(err_msg);
            case DS5:
                //val = dualsenseCotroller.getR1Button();
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.getRightBumperButton();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Whether the back button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getBackButtonPressed() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                //val = dualshockController.getR1Button();
                throw new UnsupportedOperationException(err_msg);
            case DS5:
                //val = dualsenseCotroller.getR1Button();
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.getRightBumperButtonPressed();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Whether the back button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getBackButtonReleased() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                //val = dualshockController.getR1Button();
                throw new UnsupportedOperationException(err_msg);
            case DS5:
                //val = dualsenseCotroller.getR1Button();
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.getRightBumperButtonReleased();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Constructs an event instance around the back button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the back button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent back(EventLoop loop) {
        BooleanEvent val;
        switch (controllerType) {
            case DS4:
                throw new UnsupportedOperationException(err_msg);
            case DS5:
                throw new UnsupportedOperationException(err_msg);
            case XBOX:
                val = xboxController.back(loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Read the value of the start button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getStartButton() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getOptionsButton();
                break;
            case DS5:
                val = dualsenseController.getOptionsButton();
                break;
            case XBOX:
                val = xboxController.getStartButton();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Whether the start button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getStartButtonPressed() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getOptionsButtonPressed();
                break;
            case DS5:
                val = dualsenseController.getOptionsButtonPressed();
                break;
            case XBOX:
                val = xboxController.getStartButtonPressed();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Whether the start button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getStartButtonReleased() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getOptionsButtonReleased();
                break;
            case DS5:
                val = dualsenseController.getOptionsButtonReleased();
                break;
            case XBOX:
                val = xboxController.getStartButtonReleased();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Constructs an event instance around the start button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the start button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent start(EventLoop loop) {
        BooleanEvent val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.options(loop);
                break;
            case DS5:
                val = dualsenseController.options(loop);
                break;
            case XBOX:
                val = xboxController.start(loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Read the value of the left stick button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getLeftStickButton() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getL3Button();
                break;
            case DS5:
                val = dualsenseController.getL3Button();
                break;
            case XBOX:
                val = xboxController.getLeftStickButton();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Whether the left stick button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getLeftStickButtonPressed() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getL3ButtonPressed();
                break;
            case DS5:
                val = dualsenseController.getL3ButtonPressed();
                break;
            case XBOX:
                val = xboxController.getLeftStickButtonPressed();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Whether the left stick button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getLeftStickButtonReleased() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getL3ButtonReleased();
                break;
            case DS5:
                val = dualsenseController.getL3ButtonReleased();
                break;
            case XBOX:
                val = xboxController.getLeftStickButtonReleased();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Constructs an event instance around the left stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the left stick button's digital signal
     *         attached to the given loop.
     */
    public BooleanEvent leftStick(EventLoop loop) {
        BooleanEvent val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.L3(loop);
                break;
            case DS5:
                val = dualsenseController.L3(loop);
                break;
            case XBOX:
                val = xboxController.leftStick(loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Read the value of the right stick button on the controller.
     *
     * @return The state of the button.
     */
    public boolean getRightStickButton() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getR3Button();
                break;
            case DS5:
                val = dualsenseController.getR3Button();
                break;
            case XBOX:
                val = xboxController.getRightStickButton();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Whether the right stick button was pressed since the last check.
     *
     * @return Whether the button was pressed since the last check.
     */
    public boolean getRightStickButtonPressed() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getR3ButtonPressed();
                break;
            case DS5:
                val = dualsenseController.getR3ButtonPressed();
                break;
            case XBOX:
                val = xboxController.getRightStickButtonPressed();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Whether the right stick button was released since the last check.
     *
     * @return Whether the button was released since the last check.
     */
    public boolean getRightStickButtonReleased() {
        boolean val = false;
        switch (controllerType) {
            case DS4:
                val = dualshockController.getR3ButtonReleased();
                break;
            case DS5:
                val = dualsenseController.getR3ButtonReleased();
                break;
            case XBOX:
                val = xboxController.getRightStickButtonReleased();
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }

    /**
     * Constructs an event instance around the right stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right stick button's digital
     *         signal
     *         attached to the given loop.
     */
    public BooleanEvent rightStick(EventLoop loop) {
        BooleanEvent val;
        switch (controllerType) {
            case DS4:
                val = dualshockController.R3(loop);
                break;
            case DS5:
                val = dualsenseController.R3(loop);
                break;
            case XBOX:
                val = xboxController.rightStick(loop);
                break;
            case OTHER:
                throw new UnsupportedOperationException(err_msg);
            default:
                throw new UnsupportedOperationException(err_msg);
        }
        return val;
    }
}
