package frc.robot.constants;

import frc.robot.util.PARTs.Classes.PARTsUnit;
import frc.robot.util.PARTs.Classes.PARTsUnit.PARTsUnitType;

public class RobotConstants {
        public static final PARTsUnit HALF_ROBOT_WIDTH = new PARTsUnit(13.5, PARTsUnitType.Inch);
        public static final PARTsUnit BUMPER_WIDTH = new PARTsUnit(4, PARTsUnitType.Inch);
        public static final PARTsUnit ROBOT_VISION_OFFSET = new PARTsUnit(
                        HALF_ROBOT_WIDTH.getValue() + BUMPER_WIDTH.getValue(), PARTsUnitType.Inch);
        public static final PARTsUnit ROBOT_VISION_L4_OFFSET = new PARTsUnit(ROBOT_VISION_OFFSET.getValue() + 4,
                        PARTsUnitType.Inch);

        public static boolean LOGGING = true;
        public static boolean DEBUGGING = true;
        public static boolean ALLOW_AUTO_CONTROLLER_DETECTION = false;
}
