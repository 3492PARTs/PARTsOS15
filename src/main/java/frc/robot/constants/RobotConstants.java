package frc.robot.constants;

import frc.robot.util.PARTsUnit;
import frc.robot.util.PARTsUnit.PARTsUnitType;

public class RobotConstants {
    public static class robotConstants {
        public static final PARTsUnit halfRobotWidth = new PARTsUnit(13.5, PARTsUnitType.Inch);
        public static final PARTsUnit bumperWidth = new PARTsUnit(4, PARTsUnitType.Inch);
        public static final PARTsUnit frontRobotVisionOffset = new PARTsUnit(
                halfRobotWidth.getValue() + bumperWidth.getValue(), PARTsUnitType.Inch);
        public static final PARTsUnit frontRobotVisionL4Offset = new PARTsUnit(frontRobotVisionOffset.getValue() + 4,
                PARTsUnitType.Inch);
    }
}
