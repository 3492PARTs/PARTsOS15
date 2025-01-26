// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.util.PartsMath;

public class Vision extends SubsystemBase {

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
    double angleToGoal = LimelightHelpers.getTY("") + Constants.VisionConstants.LIMELIGHT_ANGLE;

    double distance = (goalHeight - Constants.VisionConstants.LIMELIGHT_LENS_HEIGHT) / Math.tan(angleToGoal * (Math.PI/180));
    return PartsMath.InchesToMeters.apply(distance);
  }

  /**
   * Gets the horizontal offset from the crosshair to the target in degrees.
   * @return Horizontal offset angle in degrees.
   */
  public double getTX() {
    return LimelightHelpers.getTX(Constants.VisionConstants.LIMELIGHT_NAME);
  }

  /**
   * Gets the vertical offset from the crosshair to the target in degrees.
   * @return Vertical offset angle in degrees.
   */
  public double getTY() {
    return LimelightHelpers.getTY(Constants.VisionConstants.LIMELIGHT_NAME);
  }

  /**
   * Gets the target area as a percentage of the image. (0% - 100%)
   * @return Limelight TA percentage as a double. (0 - 100)
   */
  public double getTA() {
    return LimelightHelpers.getTA(Constants.VisionConstants.LIMELIGHT_NAME);
  }

  /**
   * Does the vision camera have a valid target?
   * @return True if a valid target is found, otherwise false.
   */
  public boolean isTarget(){
     return LimelightHelpers.getTV(Constants.VisionConstants.LIMELIGHT_NAME);
  }

  /**
   * Switch the pipeline via the index in the limelight.
   * @param index The index of the pipeline to set.
   */
  public void setPipelineIndex(int index) {
    LimelightHelpers.setPipelineIndex(Constants.VisionConstants.LIMELIGHT_NAME, index);
  }
}
