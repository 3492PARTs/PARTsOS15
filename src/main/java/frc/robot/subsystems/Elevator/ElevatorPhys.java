// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.Candle;

/** Add your docs here. */
public class ElevatorPhys extends Elevator {
    private final DigitalInput lowerLimitSwitch;

    protected final SparkMax mRightMotor;
    protected SparkMax mLeftMotor;

    protected final RelativeEncoder mLeftEncoder;
    protected final RelativeEncoder mRightEncoder;

    public ElevatorPhys(Candle candle) {
        super(candle);

        lowerLimitSwitch = new DigitalInput(ElevatorConstants.L_SWITCH_PORT);

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
    }

    //OVERRRIDE

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();

        super.partsNT.putDouble("Current/Left", mLeftMotor.getOutputCurrent());
        super.partsNT.putDouble("Current/Right", mRightMotor.getOutputCurrent());

        super.partsNT.putDouble("Output/Left", mLeftMotor.getAppliedOutput());
        super.partsNT.putDouble("Output/Right", mRightMotor.getAppliedOutput());
    }

    @Override
    public double getElevatorPosition() {
        return mLeftEncoder.getPosition();
    }

    @Override
    public double getRPS() {
        return mLeftEncoder.getVelocity() * 60 / ElevatorConstants.gearRatio; // 16 is the gear reduction
    }

    @Override
    protected void setSpeedWithoutLimits(double speed) {
        mLeftMotor.set(speed);
    }

    @Override
    protected void setSpeedVoltageLimits(double voltage) {
        mLeftMotor.setVoltage(voltage);
    }

    @Override
    protected void resetEncoder() {
        mLeftEncoder.setPosition(0.0);
    }

    @Override
    public boolean getBottomLimit() {
        if (!lowerLimitSwitch.get()) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public boolean getTopLimit() {
        return mPeriodicIO.useLaserCan && mPeriodicIO.elevator_measurement != null
                ? mPeriodicIO.elevator_measurement.distance_mm <= ElevatorConstants.maxLaserCanHeight
                : getElevatorPosition() >= ElevatorConstants.maxHeight;
    }

}
