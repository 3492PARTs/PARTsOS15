// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.PARTs.Classes.Abstracts;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;

import frc.robot.Robot;
import frc.robot.constants.CandleConstants;
import frc.robot.util.PARTs.Classes.PARTsCommandUtils;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class PARTsCandle extends PARTsSubsystem {
    // https://github.com/CrossTheRoadElec/Phoenix5-Examples/blob/master/Java%20General/CANdle%20MultiAnimation/src/main/java/frc/robot/subsystems/CANdleSystem.java
    private static CANdle candle;
    private final int LED_LENGTH = CandleConstants.ledLength;
    private Animation animation = null;

    public enum Color {

        // Basic Colors
        RED(255, 0, 0),
        ORANGE(255, 165, 0),
        YELLOW(255, 255, 0),
        GREEN(0, 255, 0),
        CYAN(0, 255, 255),
        BLUE(0, 0, 255),
        MAGENTA(255, 0, 255),
        PURPLE(128, 0, 128),
        WHITE(255, 255, 255),
        BLACK(0, 0, 0),
        GRAY(128, 128, 128),
        LIGHT_GRAY(211, 211, 211),
        DARK_GRAY(169, 169, 169),

        // Red and Pink Tones
        LIGHT_RED(255, 153, 153),
        DARK_RED(139, 0, 0),
        CRIMSON(220, 20, 60),
        SALMON(250, 128, 114),
        LIGHT_SALMON(255, 160, 122),
        DARK_SALMON(233, 150, 122),
        PINK(255, 192, 203),
        LIGHT_PINK(255, 182, 193),
        HOT_PINK(255, 105, 180),
        DEEP_PINK(255, 20, 147),
        MEDIUM_VIOLET_RED(199, 21, 133),

        // Orange and Brown Tones
        CORAL(255, 127, 80),
        TOMATO(255, 99, 71),
        ORANGE_RED(255, 69, 0),
        GOLD(255, 215, 0),
        BROWN(165, 42, 42),
        SIENNA(160, 82, 45),
        CHOCOLATE(210, 105, 30),
        PERU(205, 133, 63),
        SANDY_BROWN(244, 164, 96),
        BEIGE(245, 245, 220),

        // Yellow and Beige Tones
        LIGHT_YELLOW(255, 255, 224),
        LEMON_CHIFFON(255, 250, 205),
        LIGHT_GOLDENROD_YELLOW(250, 250, 210),
        KHAKI(240, 230, 140),
        PALE_GOLDENROD(238, 232, 170),

        // Green Tones
        LIME(0, 255, 0),
        LIME_GREEN(50, 205, 50),
        LIGHT_GREEN(144, 238, 144),
        PALE_GREEN(152, 251, 152),
        DARK_GREEN(0, 100, 0),
        FOREST_GREEN(34, 139, 34),
        SEA_GREEN(60, 179, 113),
        MEDIUM_SEA_GREEN(60, 179, 113),
        LIGHT_SEA_GREEN(32, 178, 170),
        SPRING_GREEN(0, 255, 127),
        MEDIUM_SPRING_GREEN(0, 250, 154),

        // Cyan and Blue Tones
        AQUA(0, 255, 255),
        LIGHT_CYAN(224, 255, 255),
        TURQUOISE(64, 224, 208),
        MEDIUM_TURQUOISE(72, 209, 204),
        DARK_TURQUOISE(0, 206, 209),
        CADET_BLUE(95, 158, 160),
        STEEL_BLUE(70, 130, 180),
        LIGHT_STEEL_BLUE(176, 196, 222),
        POWDER_BLUE(176, 224, 230),
        LIGHT_BLUE(173, 216, 230),
        DEEP_SKY_BLUE(0, 191, 255),
        SKY_BLUE(135, 206, 235),
        MEDIUM_BLUE(0, 0, 205),
        DARK_BLUE(0, 0, 139),
        NAVY(0, 0, 128),
        MIDNIGHT_BLUE(25, 25, 112),

        // Purple and Violet Tones
        VIOLET(238, 130, 238),
        PLUM(221, 160, 221),
        ORCHID(218, 112, 214),
        MEDIUM_ORCHID(186, 85, 211),
        DARK_ORCHID(153, 50, 204),
        DARK_VIOLET(148, 0, 211),
        INDIGO(75, 0, 130),
        MEDIUM_PURPLE(147, 112, 219),
        SLATE_BLUE(106, 90, 205),
        DARK_SLATE_BLUE(72, 61, 139),
        LAVENDER(230, 230, 250),
        THISTLE(216, 191, 216);

        public int r;
        public int g;
        public int b;

        Color(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    /** Creates a new light. */
    public PARTsCandle(String className) {
        super(className);
        candle = new CANdle(CandleConstants.candleId, "rio");

        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.Modulated; // does this do anything?
        candle.configAllSettings(configAll, 100);

        setNoColor();
    }

    /*---------------------------------- Custom Private Functions ---------------------------------*/
    protected void setColor(Color color) {
        animation = null;
        candle.animate(animation);
        candle.setLEDs(color.r, color.g, color.b);
    }

    protected void setNoColor() {
        setColor(Color.BLACK);
    }

    protected Command setColorGreenCommand() {
        return PARTsCommandUtils.setCommandName("CANdleColorGreen", runOnce(() -> setColor(Color.GREEN)));
    }

    protected Command setNoColorCommand() {
        return PARTsCommandUtils.setCommandName("CANdleColorOff", runOnce(() -> setNoColor()));
    }

    private FireAnimation getBurnyBurnAnimation() {
        return new FireAnimation(1, .5, LED_LENGTH, .5, .5);
    }

    private RainbowAnimation getRainbowAnimation() {
        return new RainbowAnimation();
    }

    private StrobeAnimation getStrobeAnimation(Color color) {
        return new StrobeAnimation(color.r, color.g, color.b);
    }

    private StrobeAnimation getStrobeAnimation(Color color, double speed) {
        return new StrobeAnimation(color.r, color.g, color.b, 0, speed, LED_LENGTH);
    }

    private SingleFadeAnimation getFadeAnimation(Color color) {
        return new SingleFadeAnimation(color.r, color.g, color.b, 0, 0.5, LED_LENGTH);
    }

    private SingleFadeAnimation getFadeAnimation(Color color, double speed) {
        return new SingleFadeAnimation(color.r, color.g, color.b, 0, speed, LED_LENGTH);
    }

    protected void runBurnyBurnAnimation() {
        setAnimation(getBurnyBurnAnimation());
    }

    protected void runRainbowAnimation() {
        setAnimation(getRainbowAnimation());
    }

    protected void runStrobeAnimation(Color color) {
        setAnimation(getStrobeAnimation(color));
    }

    protected void runStrobeAnimation(Color color, double speed) {
        setAnimation(getStrobeAnimation(color, speed));
    }

    protected void runLarsonAnimation(Color color) {
        setAnimation(getLarsonAnimation(color));
    }

    protected void runLarsonAnimation(Color color, double speed, LarsonAnimation.BounceMode bounceMode, int size) {
        setAnimation(getLarsonAnimation(color, speed, bounceMode, size));
    }

    protected void runTwinkleAnimation(Color color) {
        setAnimation(getTwinkleAnimation(color));
    }

    protected void runColorFlowAnimation(Color color) {
        setAnimation(getColorFlowAnimation(color));
    }

    protected void runColorFlowAnimation(Color color, double speed, Direction direction, int offset) {
        setAnimation(getColorFlowAnimation(color, speed, direction, offset));
    }

    protected void runFireAnimation() {
        setAnimation(getFireAnimation());
    }

    protected void runFireAnimation(double brightness, double speed, double sparking, double cooling) {
        setAnimation(getFireAnimation(brightness, speed, sparking, cooling));
    }

    protected void runTwinkleAnimation(Color color, double speed, TwinklePercent twinklePercent, int offset) {
        setAnimation(getTwinkleAnimation(color, speed, twinklePercent, offset));
    }

    protected void runFadeAnimation(Color color, double speed) {
        setAnimation(getFadeAnimation(color, speed));
    }

    private LarsonAnimation getLarsonAnimation(Color color) {
        return new LarsonAnimation(color.r, color.g, color.b);
    }

    private LarsonAnimation getLarsonAnimation(Color color, double speed, LarsonAnimation.BounceMode bounceMode,
            int size) {
        // Size max is 7
        return new LarsonAnimation(color.r, color.g, color.b, 0, speed, LED_LENGTH, bounceMode, size);
    }

    private TwinkleAnimation getTwinkleAnimation(Color color) {
        return new TwinkleAnimation(color.r, color.g, color.b);
    }

    private TwinkleAnimation getTwinkleAnimation(Color color, double speed, TwinklePercent twinklePercent, int offset) {
        return new TwinkleAnimation(color.r, color.g, color.b, 0, speed, LED_LENGTH, twinklePercent, offset);
    }

    private ColorFlowAnimation getColorFlowAnimation(Color color) {
        return new ColorFlowAnimation(color.r, color.g, color.b);
    }

    private ColorFlowAnimation getColorFlowAnimation(Color color, double speed, Direction direction, int offset) {
        return new ColorFlowAnimation(color.r, color.g, color.b, 0, speed, LED_LENGTH, direction, offset);
    }

    private FireAnimation getFireAnimation() {
        return new FireAnimation();
    }

    private FireAnimation getFireAnimation(double brightness, double speed, double sparking, double cooling) {
        return new FireAnimation(brightness, speed, LED_LENGTH, sparking, cooling);
    }

    private void setAnimation(Animation a) {
        animation = a;
    }

    /*---------------------------------- Custom Public Functions ----------------------------------*/

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
        if (Robot.isReal() && animation != null) {
            candle.animate(animation);
        }
    }

    @Override
    public void outputTelemetry() {        
        if (animation != null)
            super.partsNT.putString("Animation",
                    animation.toString().replace("com.ctre.phoenix.led.", "").replaceAll("@(\\d|\\w)*", ""));
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'reset'");
    }

    @Override
    public void log() {
        // TODO Auto-generated method stub
        // throw new UnsupportedOperationException("Unimplemented method 'log'");
    }
}
