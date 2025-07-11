package frc.robot.constants;

import frc.robot.util.PARTs.Classes.PARTsUnit;
import frc.robot.util.PARTs.Classes.PARTsUnit.PARTsUnitType;

public class RobotConstants {
        public static final PARTsUnit halfRobotWidth = new PARTsUnit(13.5, PARTsUnitType.Inch);
        public static final PARTsUnit bumperWidth = new PARTsUnit(4, PARTsUnitType.Inch);
        public static final PARTsUnit frontRobotVisionOffset = new PARTsUnit(
                        halfRobotWidth.getValue() + bumperWidth.getValue(), PARTsUnitType.Inch);
        public static final PARTsUnit frontRobotVisionL4Offset = new PARTsUnit(frontRobotVisionOffset.getValue() + 4,
                        PARTsUnitType.Inch);

        public static boolean logging = true;
        public static boolean debug = true;
        public static boolean allowAutoControllerDetection = true;
}
