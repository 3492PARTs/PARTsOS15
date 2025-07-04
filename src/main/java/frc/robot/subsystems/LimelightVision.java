package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.constants.CameraConstants;
import frc.robot.constants.CameraConstants.Camera;
import frc.robot.constants.CameraConstants.CameraName;
import frc.robot.constants.VisionConstants.visionConstants;
import frc.robot.util.Field;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.PARTsSubsystem;

public class LimelightVision extends PARTsSubsystem{

    private PARTsDrivetrain drivetrain;

    public enum MegaTagMode {
        MEGATAG1,
        MEGATAG2
    }

    public enum WhitelistMode {
        BLUE_REEF_TAGS(Field.BLUE_REEF_TAG_IDS),
        RED_REEF_TAGS(Field.RED_REEF_TAG_IDS);

        private int[] ids;

        private WhitelistMode(int... ids){
            this.ids = ids;
        }

        public int[] getIds() {
            return this.ids;
        }
    }

    private MegaTagMode megaTagMode;
    private WhitelistMode whitelistMode;
    private int imuMode;
    private int maxTagCount;

    public LimelightVision(PARTsDrivetrain drivetrain) {
        super("LimelightVision");
        this.drivetrain = drivetrain;
        for (Camera camera : CameraConstants.LimelightCameras) {
            Pose3d robotRelativePose = camera.getLocation();
            LimelightHelpers.setCameraPose_RobotSpace(
                camera.getName(), 
                robotRelativePose.getX(), 
                -robotRelativePose.getY(), 
                robotRelativePose.getZ(), 
                Units.radiansToDegrees(robotRelativePose.getRotation().getX()), 
                Units.radiansToDegrees(robotRelativePose.getRotation().getY()), 
                Units.radiansToDegrees(robotRelativePose.getRotation().getZ())
            );
        }

        maxTagCount = 0;

        setMegaTagMode(MegaTagMode.MEGATAG2);
        //setWhitelistMode(WhitelistMode.BLUE_REEF_TAGS);
        setIMUMode(1);
    }

    public void setMegaTagMode(MegaTagMode mode) {
        this.megaTagMode = mode;
        switch (mode) {
            case MEGATAG1:
                drivetrain.setVisionMeasurementStdDevs(visionConstants.MT1_STDEVS);
                break;
            case MEGATAG2:
                drivetrain.setVisionMeasurementStdDevs(visionConstants.MT2_STDEVS);
                break;
        }
    }

    public void setWhitelistMode(WhitelistMode mode) {
        this.whitelistMode = mode;
        switch (mode) {
            case BLUE_REEF_TAGS:
                setTagWhitelist(Field.BLUE_REEF_TAG_IDS);
                break;
            case RED_REEF_TAGS:
                setTagWhitelist(Field.RED_REEF_TAG_IDS);
                break;
        }
    }

    public WhitelistMode getWhitelistMode() {
        return this.whitelistMode;
    }

    private void setTagWhitelist(int... ids) {
        for (Camera camera : CameraConstants.LimelightCameras) {
            LimelightHelpers.SetFiducialIDFiltersOverride(camera.getName(), ids);
        }
    }

    public void setIMUMode(int mode) {
        this.imuMode = mode;
        for (Camera camera : CameraConstants.LimelightCameras) {
            LimelightHelpers.SetIMUMode(camera.getName(), mode);
        }
    }

    public int getMaxTagCount() {
        return this.maxTagCount;
    }

    public MegaTagMode getMTmode() {
        return megaTagMode;
    }

    public PoseEstimate getMegaTag1PoseEstimate(String limelightName) {
        return Robot.isBlue() 
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName)
            : LimelightHelpers.getBotPoseEstimate_wpiRed(limelightName);
    }

    private PoseEstimate getMegaTag2PoseEstimate(String limelightName) {
        return Robot.isBlue() 
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName)
            : LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);
    }

    private boolean robotIsOnBlueSide() {
        Pose2d pose = drivetrain.getPose();
        return pose.getX() < Field.LENGTH / 2 == Robot.isBlue();
    }

    private void updateWhitelistMode() {
        if (robotIsOnBlueSide() && getWhitelistMode() == WhitelistMode.RED_REEF_TAGS) {
            setWhitelistMode(WhitelistMode.BLUE_REEF_TAGS);
        }
        if (!robotIsOnBlueSide() && getWhitelistMode() == WhitelistMode.BLUE_REEF_TAGS) {
            setWhitelistMode(WhitelistMode.RED_REEF_TAGS);
        }
    }
    public static boolean cameraSeesTag(String cameraName) {
        return LimelightHelpers.getTV(cameraName);
    }
    public static int getVisibleTagId(String cameraName) {
        try{
            double[] targetArray = LimelightHelpers.getT2DArray(cameraName);
            return (int)targetArray[9];
        }
        catch(ArrayIndexOutOfBoundsException a) {
            return -1;
        }
    }

    @Override
    public void periodic() {
        this.maxTagCount = 0;
        partsNT.setString("Megatag Mode", megaTagMode.name());

        //updateWhitelistMode();

        for (Camera camera : CameraConstants.LimelightCameras) {
            LimelightHelpers.SetRobotOrientation(
                camera.getName(), 
                (drivetrain.getPose().getRotation().getDegrees() + (Robot.isBlue() ? 0 : 180)) % 360, 
                0, 
                0, 
                0, 
                0, 
                0
            );
            if (camera.isEnabled()) {
                PoseEstimate poseEstimate = (megaTagMode == MegaTagMode.MEGATAG2)
                    ? getMegaTag2PoseEstimate(camera.getName())
                    : getMegaTag1PoseEstimate(camera.getName());
                
                if (poseEstimate != null && poseEstimate.tagCount > 0) {
                    //System.out.println("hi" + poseEstimate.pose);
                    drivetrain.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
                    SmartDashboard.putBoolean("Vision/" + camera.getName() + "/Has Data", true);
                    SmartDashboard.putNumber("Vision/" + camera.getName() + "/Tag Count", poseEstimate.tagCount);
                    maxTagCount = Math.max(maxTagCount, poseEstimate.tagCount);
                }
                else {
                    SmartDashboard.putBoolean("Vision/" + camera.getName() + "/Has Data", false);
                    SmartDashboard.putNumber("Vision/" + camera.getName() + "/Tag Count", 0);
                }
            }
        }

        SmartDashboard.putString("Vision/Megatag Mode", getMTmode().toString());
        //SmartDashboard.putString("Vision/Whitelist Mode", getWhitelistMode().toString());
        SmartDashboard.putNumber("Vision/IMU Mode", imuMode);
    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub
        //throw new UnsupportedOperationException("Unimplemented method 'outputTelemetry'");
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        //throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
        //throw new UnsupportedOperationException("Unimplemented method 'reset'");
    }

    @Override
    public void log() {
        // TODO Auto-generated method stub
        //throw new UnsupportedOperationException("Unimplemented method 'log'");
    }
    public void resetPose() {
        setMegaTagMode(MegaTagMode.MEGATAG1);
        for (Camera camera : CameraConstants.LimelightCameras) {
            LimelightHelpers.SetRobotOrientation(
                camera.getName(), 
                (drivetrain.getPose().getRotation().getDegrees() + (Robot.isBlue() ? 0 : 180)) % 360, 
                0, 
                0, 
                0, 
                0, 
                0
            );
            if (camera.isEnabled()) {
                PoseEstimate poseEstimate = (megaTagMode == MegaTagMode.MEGATAG2)
                    ? getMegaTag2PoseEstimate(camera.getName())
                    : getMegaTag1PoseEstimate(camera.getName());
                
                if (poseEstimate != null && poseEstimate.tagCount > 0) {
                    drivetrain.resetPose(poseEstimate.pose);
                    partsNT.setBoolean(camera.getName() + "/Has Data", true);
                    partsNT.setInteger(camera.getName() + "/Tag Count", poseEstimate.tagCount);
                    maxTagCount = Math.max(maxTagCount, poseEstimate.tagCount);
                }
                else {
                    partsNT.setBoolean(camera.getName() + "/Has Data", false);
                    partsNT.setInteger(camera.getName() + "/Tag Count", 0);
                }
            }
        }
        setMegaTagMode(MegaTagMode.MEGATAG2);
    }
}