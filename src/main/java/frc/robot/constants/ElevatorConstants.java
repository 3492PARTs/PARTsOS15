package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
    public static final int LEFT_MOTOR_CAN_ID = 2;
    public static final int RIGHT_MOTOR_CAN_ID = 1;
    public static final int LASER_CAN_ID = 32;
    public static final int STOW_HEIGHT = 0;
    public static final double L2_HEIGHT = 10.6;// 8.785;
    public static final double L3_HEIGHT = 34.1;// 31.9;
    public static final double L4_HEIGHT = 74;// 73.38;//73.16;//73.80; //best so far 74.07373046875,
                                              // 73.59748840332031
    public static final double LOW_ALGAE_HEIGHT = 33.00;// 29.47;
    public static final double HIGH_ALGAE_HEIGHT = 58;// 55.548;
    public static final int LIMIT_SWITCH_PORT = 0;
    public static final double P = 1;
    public static final double I = 0;
    public static final double D = 0;
    public static final double TOLERANCE = 0.25;
    public static final int I_ZONE = 0;
    public static final int MAX_CURRENT = 40;
    public static final int MAX_VELOCITY = 100;
    public static final int MAX_ACCELERATION = 300;
    public static final double G = 0.18162;// 0.3766;
    public static final double S = 0.22467;// 0.24105;
    public static final double V = 0.020157;// 0.020009;
    public static final double A = 0.0022691;// 0.0022395;
    public static final double MAX_SPEED = 0.5;
    public static final double MAX_HEIGHT = 74.0;
    public static final double MAX_LASER_CAN_HEIGHT = 27;// 40;
    public static final double GEAR_RATIO = 16 / 1;
    public static final double HOMING_SPEED = -0.1;
    public static final double BOTTOM_LIMIT_POSITION_ERROR_MARGIN = 30;

    public static final double MIN_HEIGHT_METERS = 0;//Units.inchesToMeters(40.85); // FROM FLOOR TO TOP OF ELEVATOR
    public static final double MAX_HEIGHT_METERS = MAX_HEIGHT;//Units.inchesToMeters(69.85); // FROM FLOOR TO TOP OF ELEVATOR

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
