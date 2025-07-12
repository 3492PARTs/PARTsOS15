// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashSet;
import java.util.Set;

import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.states.CandleState;
import frc.robot.util.PARTs.Classes.PARTsCommandUtils;
import frc.robot.util.PARTs.Classes.Abstracts.PARTsCandle;

/** Add your docs here. */
public class Candle extends PARTsCandle {
    private CandleState candleState = CandleState.DISABLED;
    private Set<CandleState> candleStates = new HashSet<>();

    public Candle() {
        super("Candle");
    }

    /*---------------------------------- Custom Public Functions ----------------------------------*/
    public void addState(CandleState state) {
        candleStates.add(state);

        setState();
    }

    public void removeState(CandleState state) {
        candleStates.remove(state);

        setState();
    }

    public Command commandAddState(CandleState state) {
        return PARTsCommandUtils.setCommandName("commandAddState",
                this.runOnce(() -> addState(state)).ignoringDisable(true));
    }

    public Command commandRemoveState(CandleState state) {
        return PARTsCommandUtils.setCommandName("commandRemoveState",
                this.runOnce(() -> removeState(state)).ignoringDisable(true));
    }

    /*---------------------------------- Custom Private Functions ---------------------------------*/
    private void setState() {

        // This picks the order of states to display
        if (candleStates.contains(CandleState.ELEVATOR_ERROR))
            candleState = CandleState.ELEVATOR_ERROR;
        else if (candleStates.contains(CandleState.CORAL_LASER_EXIT_ERROR))
            candleState = CandleState.CORAL_LASER_EXIT_ERROR;
        else if (candleStates.contains(CandleState.CORAL_LASER_ENTRY_ERROR))
            candleState = CandleState.CORAL_LASER_ENTRY_ERROR;
        else if (candleStates.contains(CandleState.DISABLED))
            candleState = CandleState.DISABLED;
        else if (candleStates.contains(CandleState.CORAL_ENTERING))
            candleState = CandleState.CORAL_ENTERING;
        else if (candleStates.contains(CandleState.AUTO_ALIGN))
            candleState = CandleState.AUTO_ALIGN;
        else if (candleStates.contains(CandleState.SCORING))
            candleState = CandleState.SCORING;
        else if (candleStates.contains(CandleState.HAS_CORAL))
            candleState = CandleState.HAS_CORAL;
        else if (candleStates.contains(CandleState.FINE_GRAIN_DRIVE))
            candleState = CandleState.FINE_GRAIN_DRIVE;
        else if (candleStates.contains(CandleState.IDLE))
            candleState = CandleState.IDLE;

        setStateAnimation();
    }

    private void setStateAnimation() {
        // Maps state to animation
        switch (candleState) {
            case ELEVATOR_ERROR:
                runLarsonAnimation(Color.ORANGE, 0.75, BounceMode.Center, 7);
                break;
            case CORAL_LASER_EXIT_ERROR:
                runLarsonAnimation(Color.RED, 0.75, BounceMode.Center, 7);
                break;
            case CORAL_LASER_ENTRY_ERROR:
                runLarsonAnimation(Color.YELLOW, 0.75, BounceMode.Center, 7);
                break;
            case FINE_GRAIN_DRIVE:
                runTwinkleAnimation(Color.ORANGE, .75, TwinklePercent.Percent30, 0);
                break;
            case CORAL_ENTERING:
                runFadeAnimation(Color.PURPLE, .75);
                break;
            case HAS_CORAL:
                runFadeAnimation(Color.GREEN, .75);
                break;
            case SCORING:
                runRainbowAnimation();
                break;
            case IDLE:
                runFadeAnimation(Color.BLUE, .75);
                break;
            case DISABLED:
                setColor(Color.BLUE);
                break;
            case AUTO_ALIGN:
                runTwinkleAnimation(Color.AQUA, .75, TwinklePercent.Percent76, 0);
                break;
            default:
                break;
        }
    }

    /*-------------------------------- Generic Subsystem Functions --------------------------------*/

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();
        super.partsNT.putString("State", candleState.toString());
    }
}
