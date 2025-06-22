package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

/** This interface stores information about each camera. */
public interface Cameras {

    public Camera[] LimelightCameras = new Camera[] {
            new Camera("limelight-slimmy",
                    new Pose3d(.270, 0, .22, // meters
                            new Rotation3d(0, 0, 0)),
                    true),
            new Camera("limelight-thereal",
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