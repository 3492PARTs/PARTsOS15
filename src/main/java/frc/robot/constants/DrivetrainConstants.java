package frc.robot.constants;

import frc.robot.util.PARTs.PARTsUnit;
import frc.robot.util.PARTs.PARTsUnit.PARTsUnitType;

public class DrivetrainConstants {
    public static class drivetrainConstants {
        public static final String canBusName = "hi";
        public static final double kMaxAngularSpeed = Math.PI / 4; // Radians
        public static final double kMaxSpeed = .5; // Meters per second

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

        public static final PARTsUnit yRControllerTolerance = new PARTsUnit(1, PARTsUnitType.Inch);
        public static final PARTsUnit xRControllerTolerance = new PARTsUnit(2, PARTsUnitType.Inch);
        public static final PARTsUnit thetaControllerTolerance = new PARTsUnit(1, PARTsUnitType.Angle);

        public static final double leftSideOffset = 5.5;

        // more positive, more to left
        public static final PARTsUnit leftAlignDistance = new PARTsUnit(6, PARTsUnitType.Inch);

        // more negative, more to right
        public static final PARTsUnit rightAlignDistance = new PARTsUnit(-6, PARTsUnitType.Inch); // -7.5

        public static final PARTsUnit poleDistanceOffset = new PARTsUnit(6, PARTsUnitType.Inch);

        public static final PARTsUnit L4XDistance = new PARTsUnit(-9 + (-4), PARTsUnitType.Inch);

        public static final PARTsUnit xZeroHoldDistance = new PARTsUnit(-9, PARTsUnitType.Inch);

        public static final double maxAlignTime = 4; // seconds
    }
}
