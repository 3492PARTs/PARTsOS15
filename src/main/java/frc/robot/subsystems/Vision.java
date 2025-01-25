// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
 //private LimelightHelpers.LimelightTarget_Detector limelightDetector;

  /** Creates a new Vision Subsystem. */
  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Gets distance of robot in meters.
   * @param goalHeight The height of the apriltag in inches.
   * @return The distance from the apriltag in meters.
   */
  public double getDistance(double goalHeight) {
    //System.out.println("goal height: " + goalHeight);
    goalHeight = 16;
    double angleToGoal = LimelightHelpers.getTY("") +8;//getTY().plus(Constants.VisionConstants.LIMELIGHT_ANGLE);
    //Distance lensHeight = Constants.VisionConstants.LIMELIGHT_LENS_HEIGHT;
    double distance = (goalHeight - 8.75) / Math.tan(angleToGoal * (Math.PI/180));
    //Distance distanceUnit = Distance.ofBaseUnits(distance, Inches);
    //System.out.println("angleToGoal: " + angleToGoal);
    //System.out.println("distance: " + distance);
    //System.out.println("distanceUnit: " + distanceUnit);
    distance /= 39.37; // Convert to meters!!!!
    return distance;
  }

  public Angle getTX() {
    return Angle.ofBaseUnits(LimelightHelpers.getTX(""), Degrees);
  }

  /*
  public Angle getTY() {
    Angle gottenAngle = new AngleUnit().of(3); //Angle.ofBaseUnits(LimelightHelpers.getTY(""), Degrees);
    System.out.println("Base TY: " + LimelightHelpers.getTY(""));
    System.out.println("Converted TY: " + gottenAngle);
    return gottenAngle;
  }
  */

  public Dimensionless getTA(){
    return Dimensionless.ofBaseUnits(LimelightHelpers.getTA(""), Percent);
  }

  public boolean isTarget(){
     return LimelightHelpers.getTV("");
  }
}
