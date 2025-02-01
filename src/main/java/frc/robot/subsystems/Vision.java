// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.util.PARTsUnit;
import frc.robot.util.PARTsUnit.PARTsUnitType;

public class Vision extends SubsystemBase {

  enum AprilTagType {
    NONE(0),
    REEF(1),
    CAGE(2),
    PROCESSOR(3),
    STATION(4);

    public final int index;

    private AprilTagType(int index) {
        this.index = index;
    }
  }

  AprilTagType[] aprilTagList = {
    AprilTagType.NONE,      AprilTagType.STATION,   AprilTagType.STATION,
    AprilTagType.PROCESSOR, AprilTagType.CAGE,      AprilTagType.CAGE,
    AprilTagType.REEF,      AprilTagType.REEF,      AprilTagType.REEF,
    AprilTagType.REEF,      AprilTagType.REEF,      AprilTagType.REEF,
    AprilTagType.STATION,   AprilTagType.STATION,   AprilTagType.CAGE,
    AprilTagType.CAGE,      AprilTagType.PROCESSOR, AprilTagType.REEF,
    AprilTagType.REEF,      AprilTagType.REEF,      AprilTagType.REEF,
    AprilTagType.REEF
  };

  private final String LIMELIGHT_NAME;
  private final double LIMELIGHT_ANGLE;
  private final double LIMELIGHT_LENS_HEIGHT;

  /**
   * Creates a new Vision subsysten instance with the following Limelight paramaters.
   * @param limelightName The name of the requested Limelight.
   * @param limelightAngle The angle of the requested Limelight.
   * @param limelightLensHeight The height of the Limelight lens from the ground.
   */
  public Vision(String limelightName, PARTsUnit limelightAngle, PARTsUnit limelightLensHeight) {
    LIMELIGHT_NAME = limelightName;
    LIMELIGHT_ANGLE = limelightAngle.to(PARTsUnitType.Angle);
    LIMELIGHT_LENS_HEIGHT = limelightLensHeight.to(PARTsUnitType.Inch);
    // Vision array of limelight data objects.
  }

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
    double angleToGoal = LimelightHelpers.getTY(LIMELIGHT_NAME) + LIMELIGHT_ANGLE;

    double distance = (goalHeight - LIMELIGHT_LENS_HEIGHT) / Math.tan(angleToGoal * (Math.PI/180));

    return new PARTsUnit(distance, PARTsUnitType.Meter);
  }

  /**
   * Gets the horizontal offset from the crosshair to the target in degrees.
   * @return Horizontal offset angle in degrees as a {@link frc.robot.util.PARTsUnit PARTsUnit}.
   */
  public PARTsUnit getTX() {
    return new PARTsUnit(LimelightHelpers.getTX(LIMELIGHT_NAME), PARTsUnitType.Angle);
  }

  /**
   * Gets the vertical offset from the crosshair to the target in degrees.
   * @return Vertical offset angle in degrees as a {@link frc.robot.util.PARTsUnit PARTsUnit}.
   */
  public PARTsUnit getTY() {
    return new PARTsUnit(LimelightHelpers.getTY(LIMELIGHT_NAME), PARTsUnitType.Angle);
  }

  /**
   * Gets the target area as a percentage of the image. (0% - 100%)
   * @return Limelight TA percentage as a {@link frc.robot.util.PARTsUnit PARTsUnit}.
   */
  public PARTsUnit getTA() {
    return new PARTsUnit(LimelightHelpers.getTA(LIMELIGHT_NAME), PARTsUnitType.Percent);
  }

  /**
   * Does the vision camera have a valid target?
   * @return True if a valid target is found, otherwise false.
   */
  public boolean isTarget(){
     return LimelightHelpers.getTV(LIMELIGHT_NAME);
  }

  /**
   * Gets the target AprilTag ID.
   * @return The target ID as a double.
   */
  public double getTargetID() {
    double[] targetArray = LimelightHelpers.getT2DArray(LIMELIGHT_NAME);
    return targetArray[9];
  }

  /**
   * Get the target AprilTag's height as a double.
   * @param targetID The target AprilTag ID.
   * @return Returns the height of the AprilTag associated the provided ID.
   */
  public double getTargetHeight(int targetID) {
    
    for (int i=1; i < aprilTagList.length; i++) {
      if (aprilTagList[i].equals(targetID)) return i;
    }
    return 0;
  }

  /**
   * Switch the pipeline via the index in the limelight.
   * @param index The index of the pipeline to set.
   */
  public void setPipelineIndex(int index) {
    LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, index);
  }
}
