// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashSet;
import java.util.Set;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;

import frc.robot.Constants;
import frc.robot.util.PARTsSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class Candle extends PARTsSubsystem {
    //https://github.com/CrossTheRoadElec/Phoenix5-Examples/blob/master/Java%20General/CANdle%20MultiAnimation/src/main/java/frc/robot/subsystems/CANdleSystem.java
    private static CANdle candle;
    private final int LED_LENGTH = Constants.Candle.ledLength;
    private Animation animation = null;

    private PeriodicIO mPeriodicIO;

    public enum Color {

        RED(254, 0, 0),
        ORANGE(254, 55, 0),
        YELLOW(254, 254, 0),
        GREEN(0, 254, 0),
        BLUE(0, 0, 254),
        PURPLE(118, 0, 254),
        WHITE(254, 254, 254),
        OFF(0, 0, 0);

        public int r;
        public int g;
        public int b;

        Color(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    public enum CandleState {
        IDLE,
        DISABLED,
        ELEVATOR_ERROR,
        CORAL_ERROR,
        FINE_GRAIN_DRIVE,
        CORAL_ENTERING,
        HAS_CORAL,
        ELEVATOR_STOW,
        ELEVATOR_L2,
        ELEVATOR_L3,
        ELEVATOR_L4,
        SCORING
    }

    private static class PeriodicIO {
        CandleState state = CandleState.DISABLED;
        Set<CandleState> robotStates = new HashSet<>();
    }

    /** Creates a new light. */
    public Candle() {
        super("Candle");
        mPeriodicIO = new PeriodicIO();
        candle = new CANdle(Constants.Candle.candleId, "rio");

        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        //TODo: See if this fixes the red and green being swapped
        //configAll.stripType = LEDStripType.RGB;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.Modulated; // does this do anything?
        candle.configAllSettings(configAll, 100);

        setColor(Color.OFF);
        publishDashboardValues();
    }

    /*---------------------------------- Custom Private Functions ---------------------------------*/
    private void setState() {
        if (mPeriodicIO.robotStates.contains(CandleState.SCORING))
            mPeriodicIO.state = CandleState.SCORING;
        else if (mPeriodicIO.robotStates.contains(CandleState.FINE_GRAIN_DRIVE))
            mPeriodicIO.state = CandleState.FINE_GRAIN_DRIVE;
        else if (mPeriodicIO.robotStates.contains(CandleState.CORAL_ENTERING))
            mPeriodicIO.state = CandleState.CORAL_ENTERING;
        else if (mPeriodicIO.robotStates.contains(CandleState.HAS_CORAL))
            mPeriodicIO.state = CandleState.HAS_CORAL;
        else if (mPeriodicIO.robotStates.contains(CandleState.ELEVATOR_ERROR))
            mPeriodicIO.state = CandleState.ELEVATOR_ERROR;
        else if (mPeriodicIO.robotStates.contains(CandleState.CORAL_ERROR))
            mPeriodicIO.state = CandleState.CORAL_ERROR;
        /*else if (mPeriodicIO.robotStates.contains(CandleState.ELEVATOR_L4))
            mPeriodicIO.state = CandleState.ELEVATOR_L4;
        else if (mPeriodicIO.robotStates.contains(CandleState.ELEVATOR_L3))
            mPeriodicIO.state = CandleState.ELEVATOR_L3;
        else if (mPeriodicIO.robotStates.contains(CandleState.ELEVATOR_L2))
            mPeriodicIO.state = CandleState.ELEVATOR_L2;
        else if (mPeriodicIO.robotStates.contains(CandleState.ELEVATOR_STOW))
            mPeriodicIO.state = CandleState.ELEVATOR_STOW;;*/
        else if (mPeriodicIO.robotStates.contains(CandleState.IDLE))
            mPeriodicIO.state = CandleState.IDLE;
        else if (mPeriodicIO.robotStates.contains(CandleState.DISABLED))
            mPeriodicIO.state = CandleState.DISABLED;

        setStateAnimation();
    }

    private void setStateAnimation() {
        switch (mPeriodicIO.state) {
            case FINE_GRAIN_DRIVE:
                runFadeAnimation(Color.YELLOW, .75);
                break;
            case CORAL_ENTERING:
                runFadeAnimation(Color.PURPLE, .75);
                break;
            case HAS_CORAL:
                runFadeAnimation(Color.GREEN, .75);
                break;
            case ELEVATOR_ERROR:
                runFadeAnimation(Color.RED, .75);
                break;
            case CORAL_ERROR:
                runBlinkAnimation(Color.ORANGE);
                break;
            case IDLE:
                runFadeAnimation(Color.BLUE, .75);
                break;
            case DISABLED:
                setColor(Color.BLUE);
                break;
            case SCORING:
                runRainbowAnimation();
                break;
            default:
                break;
        }

        publishDashboardValues();
    }

    private void setColor(Color color) {
        animation = null;
        candle.animate(animation);
        candle.setLEDs(color.r, color.g, color.b);
    }

    private void setNoColor() {
        setColor(Color.OFF);
    }

    private Command setColorGreenCommand() {
        return super.commandFactory("CANdleColorGreen", runOnce(() -> setColor(Color.GREEN)));
    }

    private Command setNoColorCommand() {
        return super.commandFactory("CANdleColorOff", runOnce(() -> setColor(Color.OFF)));
    }

    private FireAnimation getBurnyBurnAnimation() {
        return new FireAnimation(1, .5, LED_LENGTH, .5, .5);
    }

    private RainbowAnimation getRainbowAnimation() {
        return new RainbowAnimation();
    }

    private StrobeAnimation getBlinkAnimation(Color color) {
        return new StrobeAnimation(color.r, color.g, color.b);
    }

    private SingleFadeAnimation getFadeAnimation(Color color) {
        return new SingleFadeAnimation(color.r, color.g, color.b, 0, 0.5, LED_LENGTH);
    }

    private SingleFadeAnimation getFadeAnimation(Color color, double speed) {
        return new SingleFadeAnimation(color.r, color.g, color.b, 0, speed, LED_LENGTH);
    }
    
    private void runBurnyBurnAnimation() {
        setAnimation(getBurnyBurnAnimation());
    }

    private void runRainbowAnimation() {
        setAnimation(getRainbowAnimation());
    }

    private void runBlinkAnimation(Color color) {
        setAnimation(getBlinkAnimation(color));
    }

    private void runFadeAnimation(Color color, double speed) {
        setAnimation(getFadeAnimation(color, speed));
    }

    private void setAnimation(Animation a) {
        animation = a;
    }

    /*---------------------------------- Custom Public Functions ----------------------------------*/
    public void addState(CandleState state) {
        mPeriodicIO.robotStates.add(state);

        setState();
    }

    public void removeState(CandleState state) {
        mPeriodicIO.robotStates.remove(state);

        setState();
    }

    public void disable() {
        mPeriodicIO.robotStates.clear();
        mPeriodicIO.robotStates.add(CandleState.DISABLED);

        setState();
    }

    public Command addStateCommand(CandleState state) {
        return super.commandFactory("addStateCommand", this.runOnce(() -> addStateCommand(state)));
    }

    public Command removeStateCommand(CandleState state) {
        return super.commandFactory("removeStateCommand", this.runOnce(() -> removeState(state)));
    }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() {
        return candle.getBusVoltage();
    }

    public double get5V() {
        return candle.get5VRailVoltage();
    }

    public double getCurrent() {
        return candle.getCurrent();
    }

    public double getTemperature() {
        return candle.getTemperature();
    }

    public void configBrightness(double percent) {
        candle.configBrightnessScalar(percent, 0);
    }

    public void configLos(boolean disableWhenLos) {
        candle.configLOSBehavior(disableWhenLos, 0);
    }

    public void configLedType(LEDStripType type) {
        candle.configLEDType(type, 0);
    }

    public void configStatusLedBehavior(boolean offWhenActive) {
        candle.configStatusLedState(offWhenActive, 0);
    }

    /*-------------------------------- Generic Subsystem Functions --------------------------------*/
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (animation != null) {
            candle.animate(animation);
        }
    }

    @Override
    public void outputTelemetry() {

    }

    private void publishDashboardValues() {
        super.partsNT.setString("State", mPeriodicIO.state.toString());
        if (animation != null)
            super.partsNT.setString("Animation", animation.toString().replace("com.ctre.phoenix.led.", ""));
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        //throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        //throw new UnsupportedOperationException("Unimplemented method 'reset'");
    }

    @Override
    public void log() {
        // TODO Auto-generated method stub
        //throw new UnsupportedOperationException("Unimplemented method 'log'");
    }
}
