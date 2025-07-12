// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Coral;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import frc.robot.constants.CoralConstants;

/** Add your docs here. */
public class CoralPhys extends Coral {

    private SparkMax mLeftMotor;
    private SparkMax mRightMotor;

    private LaserCan entrySensor;
    private LaserCan exitSensor;

    public CoralPhys() {
        super();

        mLeftMotor = new SparkMax(CoralConstants.coralLeftMotorId, MotorType.kBrushless);
        mRightMotor = new SparkMax(CoralConstants.coralRightMotorId, MotorType.kBrushless);

        SparkMaxConfig coralConfig = new SparkMaxConfig();

        coralConfig.idleMode(IdleMode.kBrake);

        mLeftMotor.configure(
                coralConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        mRightMotor.configure(
                coralConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        entrySensor = new LaserCan(CoralConstants.laserCanId);
        try {
            entrySensor.setRangingMode(LaserCan.RangingMode.SHORT);
            entrySensor.setRegionOfInterest(new LaserCan.RegionOfInterest(4, 4, 4, 4));
            entrySensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }

        exitSensor = new LaserCan(CoralConstants.laserCan2Id);
        try {
            exitSensor.setRangingMode(LaserCan.RangingMode.SHORT);
            exitSensor.setRegionOfInterest(new LaserCan.RegionOfInterest(4, 4, 4, 4));
            exitSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        entryLaserMeasurement = entrySensor.getMeasurement();
        exitLaserMeasurement = exitSensor.getMeasurement();

        mLeftMotor.set(coralState.getSpeed() - coralState.getSpeedDiff());
        mRightMotor.set(-coralState.getSpeed());
    }

    @Override
    public void outputTelemetry() {
        super.outputTelemetry();

        super.partsNT.putDouble("Current/Left", mLeftMotor.getOutputCurrent());
        super.partsNT.putDouble("Current/Right", mRightMotor.getOutputCurrent());

        super.partsNT.putDouble("Output/Left", mLeftMotor.getAppliedOutput());
        super.partsNT.putDouble("Output/Right", mRightMotor.getAppliedOutput());
    }
}
