package frc.robot.constants;

import frc.robot.util.PARTs.Classes.PARTsUnit;
import frc.robot.util.PARTs.Classes.PARTsUnit.PARTsUnitType;

public class DrivetrainConstants {
        public static final String CAN_BUS_NAME = "hi";
        public static final double MAX_ANGULAR_SPEED = Math.PI / 4; // Radians
        public static final double MAX_SPEED = .5; // Meters per second

        public static final double MAX_AIM_VELOCITY = 1.5 * Math.PI; // radd/s
        public static final double MAX_AIM_ACCELERATION = Math.PI / 2; // rad/s^2
        public static final double MAX_RANGE_VELOCITY = 2;// m/s
        public static final double MAX_RANGE_ACCELERATION = 2;// m/2^s

        public static final double THETA_P = 5; // Proprotinal //4.5
        public static final double THETA_I = 0; // Gradual corretction
        public static final double THETA_D = 0; // Smooth oscilattions

        public static final double RANGE_X_P = 10; // 6.5
        public static final double RANGE_I = 0;
        public static final double RANGE_D = 0;

        public static final double RANGE_Y_P = 4.5; // 4.5

        public static final PARTsUnit Y_RANGE_CONTROLLER_TOLERANCE = new PARTsUnit(1, PARTsUnitType.Inch);
        public static final PARTsUnit X_RANGE_CONTROLLER_TOLERANCE = new PARTsUnit(2, PARTsUnitType.Inch);
        public static final PARTsUnit THETA_CONTROLLER_TOLERANCE = new PARTsUnit(1, PARTsUnitType.Angle);

        public static final double LEFT_SIDE_OFFSET = 5.5;

        // more positive, more to left
        public static final PARTsUnit LEFT_ALIGN_DISTANCE = new PARTsUnit(6, PARTsUnitType.Inch);

        // more negative, more to right
        public static final PARTsUnit RIGHT_ALIGN_DISTANCE = new PARTsUnit(-6, PARTsUnitType.Inch); // -7.5

        public static final PARTsUnit POLE_DISTANCE_OFFSET = new PARTsUnit(6, PARTsUnitType.Inch);

        public static final PARTsUnit L4_X_DISTANCE = new PARTsUnit(-9 + (-4), PARTsUnitType.Inch);

        public static final PARTsUnit X_ZERO_HOLD_DISTANCE = new PARTsUnit(-9, PARTsUnitType.Inch);

        public static final double ALIGN_TIMEOUT = 0.25; // seconds
}
