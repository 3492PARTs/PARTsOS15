package frc.robot.subsystems;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.CameraConstants;
import frc.robot.constants.CameraConstants.Camera;
import frc.robot.constants.VisionConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Field.Field;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.PARTs.Classes.PARTsCommandUtils;
import frc.robot.util.PARTs.Classes.Abstracts.PARTsSubsystem;

public class LimelightVision extends PARTsSubsystem {

    private final Supplier<Pose2d> poseSupplier;
    private final BiConsumer<Pose2d, Double> addVisionMeasurementBiConsumer;
    private final Consumer<Vector<N3>> setVisionMeasurementStdDevsConsumer;
    private final Consumer<Pose2d> resetPoseConsumer;

    public enum MegaTagMode {
        MEGATAG1,
        MEGATAG2
    }

    public enum WhitelistMode {
        BLUE_REEF_TAGS(Field.BLUE_REEF_TAG_IDS),
        RED_REEF_TAGS(Field.RED_REEF_TAG_IDS),
        ALL(Field.getAllTagIDs()),
        NONE(new int[0]);

        private int[] ids;

        private WhitelistMode(int... ids) {
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

    public LimelightVision(Supplier<Pose2d> poseSupplier, BiConsumer<Pose2d, Double> addVisionMeasurementBiConsumer,
            Consumer<Vector<N3>> setVisionMeasurementStdDevsConsumer, Consumer<Pose2d> resetPoseConsumer) {
        super("LimelightVision");
        this.poseSupplier = poseSupplier;
        this.addVisionMeasurementBiConsumer = addVisionMeasurementBiConsumer;
        this.setVisionMeasurementStdDevsConsumer = setVisionMeasurementStdDevsConsumer;
        this.resetPoseConsumer = resetPoseConsumer;
        for (Camera camera : CameraConstants.LimelightCameras) {
            Pose3d robotRelativePose = camera.getLocation();
            LimelightHelpers.setCameraPose_RobotSpace(
                    camera.getName(),
                    robotRelativePose.getX(),
                    -robotRelativePose.getY(),
                    robotRelativePose.getZ(),
                    Units.radiansToDegrees(robotRelativePose.getRotation().getX()),
                    Units.radiansToDegrees(robotRelativePose.getRotation().getY()),
                    Units.radiansToDegrees(robotRelativePose.getRotation().getZ()));
        }

        maxTagCount = 0;

        setMegaTagMode(MegaTagMode.MEGATAG2);
        setWhitelistMode(WhitelistMode.ALL);
        setIMUMode(1);

        //elastic crashes :(
        //super.partsNT.putSmartDashboardSendable("Set MT-1", commandMegaTagMode(MegaTagMode.MEGATAG1));
        //super.partsNT.putSmartDashboardSendable("Set MT-2", commandMegaTagMode(MegaTagMode.MEGATAG2));
    }

    public void setMegaTagMode(MegaTagMode mode) {
        this.megaTagMode = mode;
        switch (mode) {
            case MEGATAG1:
                setVisionMeasurementStdDevsConsumer.accept(VisionConstants.MT1_STDEVS);
                break;
            case MEGATAG2:
                setVisionMeasurementStdDevsConsumer.accept(VisionConstants.MT2_STDEVS);
                break;
        }
    }

    public Command commandMegaTagMode(MegaTagMode mode) {
        Command c = PARTsCommandUtils.setCommandName("commandMegaTagMode", this.runOnce(() -> setMegaTagMode(mode)));
        c = c.ignoringDisable(true);
        return c;
    }

    public void setWhitelistMode(WhitelistMode mode) {
        this.whitelistMode = mode;
        switch (mode) {
            case BLUE_REEF_TAGS:
                setTagWhitelist(WhitelistMode.BLUE_REEF_TAGS.getIds());
                break;
            case RED_REEF_TAGS:
                setTagWhitelist(WhitelistMode.RED_REEF_TAGS.getIds());
                break;
            case ALL:
                setTagWhitelist(WhitelistMode.ALL.getIds());
                break;
            case NONE:
                setTagWhitelist(WhitelistMode.NONE.getIds());
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
        Pose2d pose = poseSupplier.get();
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
        try {
            double[] targetArray = LimelightHelpers.getT2DArray(cameraName);
            return (int) targetArray[9];
        } catch (ArrayIndexOutOfBoundsException a) {
            return -1;
        }
    }

    @Override
    public void periodic() {
        this.maxTagCount = 0;

        //updateWhitelistMode();

        for (Camera camera : CameraConstants.LimelightCameras) {
            LimelightHelpers.SetRobotOrientation(
                    camera.getName(),
                    (poseSupplier.get().getRotation().getDegrees() + (Robot.isBlue() ? 0 : 180)) % 360,
                    0,
                    0,
                    0,
                    0,
                    0);
            if (camera.isEnabled()) {
                PoseEstimate poseEstimate = (megaTagMode == MegaTagMode.MEGATAG2)
                        ? getMegaTag2PoseEstimate(camera.getName())
                        : getMegaTag1PoseEstimate(camera.getName());

                if (poseEstimate != null && poseEstimate.tagCount > 0) {
                    addVisionMeasurementBiConsumer.accept(poseEstimate.pose, poseEstimate.timestampSeconds);

                    partsNT.putBoolean(camera.getName() + "/Has Data", true);
                    partsNT.putNumber(camera.getName() + "/Tag Count", poseEstimate.tagCount);

                    maxTagCount = Math.max(maxTagCount, poseEstimate.tagCount);
                } else {
                    partsNT.putBoolean(camera.getName() + "/Has Data", false);
                    partsNT.putNumber(camera.getName() + "/Tag Count", 0);
                }
            }
        }
    }

    @Override
    public void outputTelemetry() {
        partsNT.putString("Megatag Mode", getMTmode().toString());
        partsNT.putString("Whitelist Mode", getWhitelistMode().toString());
        partsNT.putNumber("IMU Mode", imuMode);
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

    public void resetRobotPose() {
        setMegaTagMode(MegaTagMode.MEGATAG1);
        for (Camera camera : CameraConstants.LimelightCameras) {
            LimelightHelpers.SetRobotOrientation(
                    camera.getName(),
                    (poseSupplier.get().getRotation().getDegrees() + (Robot.isBlue() ? 0 : 180)) % 360,
                    0,
                    0,
                    0,
                    0,
                    0);
            if (camera.isEnabled()) {
                PoseEstimate poseEstimate = (megaTagMode == MegaTagMode.MEGATAG2)
                        ? getMegaTag2PoseEstimate(camera.getName())
                        : getMegaTag1PoseEstimate(camera.getName());

                if (poseEstimate != null && poseEstimate.tagCount > 0) {
                    resetPoseConsumer.accept(poseEstimate.pose);
                    partsNT.putBoolean(camera.getName() + "/Has Data", true);
                    partsNT.putInteger(camera.getName() + "/Tag Count", poseEstimate.tagCount);
                    maxTagCount = Math.max(maxTagCount, poseEstimate.tagCount);
                } else {
                    partsNT.putBoolean(camera.getName() + "/Has Data", false);
                    partsNT.putInteger(camera.getName() + "/Tag Count", 0);
                }
            }
        }
        setMegaTagMode(MegaTagMode.MEGATAG2);
    }
}