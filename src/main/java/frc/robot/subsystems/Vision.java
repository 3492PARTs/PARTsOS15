// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
 private LimelightHelpers.LimelightTarget_Detector limelightDetector;

  /** Creates a new Vision Subsystem. */
  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Distance getDistance(double goalHeight) {
    Angle angleToGoal = getTY().plus(Constants.VisionConstants.LIMELIGHT_ANGLE);
    Distance lensHeight = Constants.VisionConstants.LIMELIGHT_LENS_HEIGHT;
    double distance = (goalHeight - lensHeight.in(Inches)) / Math.tan(angleToGoal.in(Radians));
    Distance distanceUnit = Distance.ofBaseUnits(distance, Inches);
    return distanceUnit;
  }

  public Angle getTX() {
    return Angle.ofBaseUnits(LimelightHelpers.getTX(""), Degrees);
  }

  public Angle getTY() {
    return Angle.ofBaseUnits(LimelightHelpers.getTY(""), Degrees);
  }

  public Dimensionless getTA(){
    return Dimensionless.ofBaseUnits(LimelightHelpers.getTA(""), Percent);
  }

  public boolean isTarget(){
     return LimelightHelpers.getTV("");
  }
}
