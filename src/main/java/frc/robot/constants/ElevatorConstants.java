package frc.robot.constants;

public class ElevatorConstants {
    public static class elevatorConstants {
        public static final int leftElevatorId = 2;
        public static final int rightElevatorId = 1;
        public static final int laserCanId = 32;
        public static final int StowHeight = 0;
        public static final double L2Height = 10.6;// 8.785;
        public static final double L3Height = 34.1;// 31.9;
        public static final double L4Height = 74;// 73.38;//73.16;//73.80; //best so far 74.07373046875,
                                                 // 73.59748840332031
        public static final double LowAlgaeHeight = 33.00;// 29.47;
        public static final double HighAlgaeHeight = 58;// 55.548;
        public static final int L_SWITCH_PORT = 0;
        public static final double kP = 1;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kTolerance = 0.25;
        public static final int kIZone = 0;
        public static final int kMaxCurrent = 40;
        public static final int kMaxVelocity = 100;
        public static final int kMaxAcceleration = 300;
        public static final double kG = 0.18162;// 0.3766;
        public static final double kS = 0.22467;// 0.24105;
        public static final double kV = 0.020157;// 0.020009;
        public static final double kA = 0.0022691;// 0.0022395;
        public static final double maxSpeed = 0.5;
        public static final double maxHeight = 74.0;
        public static final double maxLaserCanHeight = 27;// 40;
        public static final double gearRatio = 16 / 1;
        public static final double homingSpeed = -0.1;
        public static final double bottomLimitPositionErrorMargin = 30;
    }
}
