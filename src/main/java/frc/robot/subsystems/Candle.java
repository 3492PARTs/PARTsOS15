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
    private PeriodicIO mPeriodicIO;
    private static class PeriodicIO {
        CandleState state = CandleState.DISABLED;
        Set<CandleState> robotStates = new HashSet<>();
    }

    public Candle() {
        super();

        mPeriodicIO = new PeriodicIO();
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

    public Command addStateCommand(CandleState state) {
        return PARTsCommandUtils.setCommandName("addStateCommand", this.runOnce(() -> addState(state)));
    }

    public Command removeStateCommand(CandleState state) {
        return PARTsCommandUtils.setCommandName("removeStateCommand", this.runOnce(() -> removeState(state)));
    }

    /*---------------------------------- Custom Private Functions ---------------------------------*/
    private void setState() {

        if (mPeriodicIO.robotStates.contains(CandleState.ELEVATOR_ERROR))
            mPeriodicIO.state = CandleState.ELEVATOR_ERROR;
        else if (mPeriodicIO.robotStates.contains(CandleState.CORAL_LASER_EXIT_ERROR))
            mPeriodicIO.state = CandleState.CORAL_LASER_EXIT_ERROR;
        else if (mPeriodicIO.robotStates.contains(CandleState.CORAL_LASER_ENTRY_ERROR))
            mPeriodicIO.state = CandleState.CORAL_LASER_ENTRY_ERROR;
        else if (mPeriodicIO.robotStates.contains(CandleState.DISABLED))
            mPeriodicIO.state = CandleState.DISABLED;
        else if (mPeriodicIO.robotStates.contains(CandleState.CORAL_ENTERING))
            mPeriodicIO.state = CandleState.CORAL_ENTERING;

        else if (mPeriodicIO.robotStates.contains(CandleState.AUTO_ALIGN))
            mPeriodicIO.state = CandleState.AUTO_ALIGN;
        else if (mPeriodicIO.robotStates.contains(CandleState.SCORING))
            mPeriodicIO.state = CandleState.SCORING;
        else if (mPeriodicIO.robotStates.contains(CandleState.HAS_CORAL))
            mPeriodicIO.state = CandleState.HAS_CORAL;
        else if (mPeriodicIO.robotStates.contains(CandleState.FINE_GRAIN_DRIVE))
            mPeriodicIO.state = CandleState.FINE_GRAIN_DRIVE;
        else if (mPeriodicIO.robotStates.contains(CandleState.IDLE))
            mPeriodicIO.state = CandleState.IDLE;

        setStateAnimation();
    }

    private void setStateAnimation() {
        switch (mPeriodicIO.state) {
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
        super.partsNT.putString("State", mPeriodicIO.state.toString());
    }
}
