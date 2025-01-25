// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
public class Coral extends SubsystemBase {

  private static SparkMax coralRightMotor = new SparkMax(Constants.Coral.coralRightMotorId, MotorType.kBrushless);
  private static RelativeEncoder coralRightMotorEncoder;

  private static SparkMax coralLeftMotor = new SparkMax(Constants.Coral.coralLeftMotorId, MotorType.kBrushless);
  private static RelativeEncoder coralLeftMotorEncoder;

  private LaserCan lc;
  private Canandcolor canandcolor;

  /** Creates a new Coral. */
  public Coral() {
     SparkMaxConfig coralRightMotorConfig = new SparkMaxConfig();
    coralRightMotorConfig.idleMode(IdleMode.kBrake);
    coralRightMotor.configure(coralRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig coralLeftMotorConfig = new SparkMaxConfig();
    coralLeftMotorConfig.idleMode(IdleMode.kBrake);
    coralLeftMotorConfig.follow(coralRightMotor);
    coralLeftMotor.configure(coralLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    coralRightMotorEncoder = coralRightMotor.getEncoder();
    coralLeftMotorEncoder = coralLeftMotor.getEncoder();

    Canandcolor canandcolor = new Canandcolor(Constants.Sensors.canAndColorId);
    lc = new LaserCan(Constants.Sensors.laserCanId);
    // Optionally initialise the settings of the LaserCAN, if you haven't already done so in GrappleHook
    try {
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  public void setSpeed(double speed) {
    coralRightMotor.set(speed);
    coralLeftMotor.set(speed);
  }

  public double getEncoderDistance() {
    return coralLeftMotorEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    LaserCan.Measurement measurement = lc.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      System.out.println("The target is " + measurement.distance_mm + "mm away!");
    } else {
      System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
      // You can still use distance_mm in here, if you're ok tolerating a clamped value or an unreliable measurement.
    }
    double proximity = canandcolor.getProximity();
    double red = canandcolor.getRed();
    double blue = canandcolor.getBlue();
    double green = canandcolor.getGreen();
    System.out.println("Proximity: " + proximity +  "\n Red Value: " + red + "\n Blue Value: " + blue + "\n Green Value: " + green);
  }
}
