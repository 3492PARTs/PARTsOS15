// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.util.AprilTagData;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.PARTsSubsystem;
import frc.robot.util.PARTsUnit;
import frc.robot.util.AprilTagData.AprilTagType;
import frc.robot.util.PARTsUnit.PARTsUnitType;

public class Vision extends PARTsSubsystem {

  /*-------------------------------- Private instance variables ---------------------------------*/
  private String LIMELIGHT_NAME;
  private double LIMELIGHT_ANGLE;
  private double LIMELIGHT_LENS_HEIGHT;

  private Pose3d currentVisionPose3d;
  private double tagID = -1;

  /**
   * Creates a new Vision subsysten instance with the following Limelight
   * paramaters.
   * 
   * @param limelightName       The name of the requested Limelight.
   * @param limelightAngle      The angle of the requested Limelight.
   * @param limelightLensHeight The height of the Limelight lens from the ground.
   */
  public Vision(String limelightName, PARTsUnit limelightAngle, PARTsUnit limelightLensHeight) {
    super("vision" + (limelightName.length() > 0 ? "_" : "") + limelightName);
    LIMELIGHT_NAME = limelightName;
    LIMELIGHT_ANGLE = limelightAngle.to(PARTsUnitType.Angle);
    LIMELIGHT_LENS_HEIGHT = limelightLensHeight.to(PARTsUnitType.Inch);
    // Vision array of limelight data objects.
  }

  /*-------------------------------- Generic Subsystem Functions --------------------------------*/
  @Override
  public void periodic() {
    if (!isTarget()) {
      tagID = -1;
      currentVisionPose3d = null;
      return;
    }

    currentVisionPose3d = getPose3d();
    tagID = getTargetID();
  }

  @Override
  public void outputTelemetry() {

    if (currentVisionPose3d != null) {
      partsNT.setDouble("tagPoseX", new PARTsUnit(currentVisionPose3d.getX(), PARTsUnitType.Meter)
          .to(PARTsUnitType.Inch));
      partsNT.setDouble("tagPoseY", new PARTsUnit(currentVisionPose3d.getY(), PARTsUnitType.Meter)
          .to(PARTsUnitType.Inch));
      partsNT.setDouble("tagPoseRot", new PARTsUnit(currentVisionPose3d.getRotation().getAngle(),
          PARTsUnitType.Radian).to(PARTsUnitType.Angle));
    }

    partsNT.setBoolean("tag", tagID > 0);
    partsNT.setDouble("tagID", tagID);
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method 'stop'");
  }

  @Override
  public void reset() {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method 'reset'");
  }

  @Override
  public void log() {
    // TODO Auto-generated method stub
    // throw new UnsupportedOperationException("Unimplemented method 'log'");
  }

  /*---------------------------------- Custom Public Functions ----------------------------------*/
  /**
   * Gets distance of robot in meters.
   * 
   * @param goalHeight The height of the apriltag in inches.
   * @return The distance from the apriltag as a {@link frc.robot.util.PARTsUnit
   *         PARTsUnit} in Meters.
   * @deprecated Please do not use this to get distance. Zero will always be
   *             returned.
   */
  public PARTsUnit getDistance(double goalHeight) {

    double angleToGoal = LimelightHelpers.getTY(LIMELIGHT_NAME);
    // System.out.println("Vision -> Angle to goal: " + angleToGoal);

    // double distance = (goalHeight - LIMELIGHT_LENS_HEIGHT) / Math.tan(angleToGoal
    // * (Math.PI/180));
    // System.out.println("Vision -> Distance: " + distance);

    // Dist from limelight, convert to x y for robot and

    PARTsUnit unit = new PARTsUnit(0, PARTsUnitType.Inch);
    return unit.as(PARTsUnitType.Meter);
  }

  public Pose3d getPose3d() {
    return LimelightHelpers.getBotPose3d_TargetSpace(LIMELIGHT_NAME);
  }

  /**
   * Gets the horizontal offset from the crosshair to the target in degrees.
   * 
   * @return Horizontal offset angle in degrees as a
   *         {@link frc.robot.util.PARTsUnit PARTsUnit}.
   */
  public PARTsUnit getTX() {
    return new PARTsUnit(LimelightHelpers.getTX(LIMELIGHT_NAME), PARTsUnitType.Angle);
  }

  /**
   * Gets the vertical offset from the crosshair to the target in degrees.
   * 
   * @return Vertical offset angle in degrees as a {@link frc.robot.util.PARTsUnit
   *         PARTsUnit}.
   */
  public PARTsUnit getTY() {
    return new PARTsUnit(LimelightHelpers.getTY(LIMELIGHT_NAME), PARTsUnitType.Angle);
  }

  /**
   * Gets the target area as a percentage of the image. (0% - 100%)
   * 
   * @return Limelight TA percentage as a {@link frc.robot.util.PARTsUnit
   *         PARTsUnit}.
   */
  public PARTsUnit getTA() {
    return new PARTsUnit(LimelightHelpers.getTA(LIMELIGHT_NAME), PARTsUnitType.Percent);
  }

  /**
   * Does the vision camera have a valid target?
   * 
   * @return True if a valid target is found, otherwise false.
   */
  public boolean isTarget() {
    return LimelightHelpers.getTV(LIMELIGHT_NAME);
  }

  /**
   * Gets the target AprilTag ID.
   * 
   * @return The target ID as a double.
   */
  public double getTargetID() {
    double[] targetArray = LimelightHelpers.getT2DArray(LIMELIGHT_NAME);
    return targetArray[9];
  }

  /**
   * Sets the Limelight's priority AprilTag ID to the requested AprilTag ID.
   * 
   * @param targetID The requested AprilTag ID.
   */
  public void setTargetID(int targetID) {
    if (AprilTagData.getTargeTagType(targetID).equals(AprilTagType.NONE))
      return;
    LimelightHelpers.setPriorityTagID(LIMELIGHT_NAME, targetID);
  }

  /**
   * Get the target AprilTag's height as a double.
   * 
   * @param targetID The target AprilTag ID.
   * @return Returns the height of the AprilTag associated the provided ID.
   */
  public double getTargetHeight(int targetID) {
    return AprilTagData.getAprilTagHeight(targetID).to(PARTsUnitType.Inch);
  }

  /**
   * Switch the pipeline via the index in the limelight.
   * 
   * @param index The index of the pipeline to set.
   */
  public void setPipelineIndex(int index) {
    LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, index);
  }

  /**
   * Coverts the pose in target space to a pose in robot space.
   * 
   * @see <a href=
   *      "https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems">3D
   *      Coordinate Systems in Detail</a>.
   * @param pose The pose in target space.
   * @return The new pose in robot space.
   */
  public Pose3d convertToKnownSpace(Pose3d pose) {
    return new Pose3d(
        pose.getZ(),
        -pose.getX(),
        pose.getY(),
        pose.getRotation() // TODO: Look into if this is an issue later.
    );
  }

  /**
   * Coverts the pose in target space to a pose in robot space.
   * 
   * @see <a href=
   *      "https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems">3D
   *      Coordinate Systems in Detail</a>.
   * @param pose The pose in target space.
   * @return The new pose in robot space.
   */
  public Pose3d convertToKnownSpace(Pose3d pose, Rotation3d rotation) {
    return new Pose3d(
        pose.getZ(),
        -pose.getX(),
        pose.getY(),
        rotation);
  }

  public Rotation2d flipRotation2d(Rotation2d rotation) {
    return new Rotation2d(-rotation.getRadians());
  }

  public Pose2d getBotRelativePose2d() {
    double[] botPoseTargetSpace = getBotPoseTargetSpace();

    Pose3d pose3d = convertToKnownSpace(currentVisionPose3d);

    double turnPosNeg = -Math.signum(botPoseTargetSpace[4]);

    Pose2d pose2d = new Pose2d(pose3d.getX(), pose3d.getY(),
        new Rotation2d(pose3d.getRotation().getAngle()
            * turnPosNeg));
    return pose2d;
  }

  public double getTagId() {
    return tagID;
  }

  /*---------------------------------- Custom Private Functions ---------------------------------*/
  private double[] getBotPoseTargetSpace() {
    return LimelightHelpers.getLimelightNTDoubleArray(LIMELIGHT_NAME,
        "botpose_targetspace");
  }
}
