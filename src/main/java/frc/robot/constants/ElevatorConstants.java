package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
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

    public static final double MIN_HEIGHT_METERS = 0;//Units.inchesToMeters(40.85); // FROM FLOOR TO TOP OF ELEVATOR
    public static final double MAX_HEIGHT_METERS = maxHeight;//Units.inchesToMeters(69.85); // FROM FLOOR TO TOP OF ELEVATOR

    public static final double MASS_KG = Units.lbsToKilograms(1); // Currently for sim only, not confirmed
    public static final double DRUM_RADIUS_METERS = ((MAX_HEIGHT_METERS - MIN_HEIGHT_METERS)
            / (Encoders.NUM_ROTATIONS_TO_REACH_TOP / Encoders.GEAR_RATIO)) / 2 / Math.PI;

    public interface Encoders {
        double GEAR_RATIO = 52.0 / 12.0;

        double NUM_ROTATIONS_TO_REACH_TOP = (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / (0.480 / 13); // Number of rotations that the motor has to spin, NOT the gear
        double POSITION_CONVERSION_FACTOR = (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / NUM_ROTATIONS_TO_REACH_TOP;
        double VELOCITY_CONVERSION_FACTOR = (MAX_HEIGHT_METERS - MIN_HEIGHT_METERS) / NUM_ROTATIONS_TO_REACH_TOP / 60;
    }
}
