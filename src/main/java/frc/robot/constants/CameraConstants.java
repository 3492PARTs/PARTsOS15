package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

//move to constants pls

/** This interface stores information about each camera. */
public interface CameraConstants {
    public enum CameraName {
        FRONT_CAMERA("limelight-slimmy"),
        BACK_CAMERA("limelight-thereal");

        private String cameraName = "";

        private CameraName(String name) {
            cameraName = name;
        }

        public String getCameraName() {
            return cameraName;
        }
    }

    public Camera[] LimelightCameras = new Camera[] {
            new Camera(CameraName.FRONT_CAMERA.getCameraName(),
                    new Pose3d(.270, 0, .22, // meters
                            new Rotation3d(0, 0, 0)),
                    true),
            new Camera(CameraName.BACK_CAMERA.getCameraName(),
                    new Pose3d(-.265, 0, .545, // meters
                            new Rotation3d(0, 0,
                                    Units.degreesToRadians(180))),
                    true)
    };

    public static class Camera {
        private String name;
        private Pose3d location;
        private Boolean isEnabled;

        public Camera(String name, Pose3d location, Boolean isEnabled) {
            this.name = name;
            this.location = location;
            this.isEnabled = isEnabled;
        }

        public String getName() {
            return name;
        }

        public Pose3d getLocation() {
            return location;
        }

        public boolean isEnabled() {
            return isEnabled;
        }

        public void setEnabled(boolean enabled) {
            this.isEnabled = enabled;
        }
    }
}