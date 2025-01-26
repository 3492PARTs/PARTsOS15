// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.util.PARTsUnit;
import frc.robot.util.PARTsUnit.PARTsUnitType;

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
   * @return The distance from the apriltag as a {@link frc.robot.util.PARTsUnit PARTsUnit} in Meters.
   */
  public PARTsUnit getDistance(double goalHeight) {
    double angleToGoal = LimelightHelpers.getTY("") + Constants.VisionConstants.LIMELIGHT_ANGLE;

    double distance = (goalHeight - Constants.VisionConstants.LIMELIGHT_LENS_HEIGHT) / Math.tan(angleToGoal * (Math.PI/180));

    return new PARTsUnit(distance, PARTsUnitType.Meter);
  }

  /**
   * Gets the horizontal offset from the crosshair to the target in degrees.
   * @return Horizontal offset angle in degrees as a {@link frc.robot.util.PARTsUnit PARTsUnit}.
   */
  public PARTsUnit getTX() {
    return new PARTsUnit(LimelightHelpers.getTX(Constants.VisionConstants.LIMELIGHT_NAME), PARTsUnitType.Angle);
  }

  /**
   * Gets the vertical offset from the crosshair to the target in degrees.
   * @return Vertical offset angle in degrees as a {@link frc.robot.util.PARTsUnit PARTsUnit}.
   */
  public PARTsUnit getTY() {
    return new PARTsUnit(LimelightHelpers.getTY(Constants.VisionConstants.LIMELIGHT_NAME), PARTsUnitType.Angle);
  }

  /**
   * Gets the target area as a percentage of the image. (0% - 100%)
   * @return Limelight TA percentage as a {@link frc.robot.util.PARTsUnit PARTsUnit}.
   */
  public PARTsUnit getTA() {
    return new PARTsUnit(LimelightHelpers.getTA(Constants.VisionConstants.LIMELIGHT_NAME), PARTsUnitType.Percent);
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
