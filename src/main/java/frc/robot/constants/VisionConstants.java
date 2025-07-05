package frc.robot.constants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N3;
import frc.robot.util.PARTs.PARTsUnit;
import frc.robot.util.PARTs.PARTsUnit.PARTsUnitType;

public class VisionConstants {
    public static class visionConstants {
        public static final String DRIVETRAIN_LIMELIGHT = "limelight-slimmy"; // The_Real
        public static final String ELEVATOR_LIMELIGHT = "limelight-thereal";
        public static final PARTsUnit LIMELIGHT_LENS_HEIGHT = new PARTsUnit(9, PARTsUnitType.Inch); // Inches
        public static final PARTsUnit LIMELIGHT_ANGLE = new PARTsUnit(0, PARTsUnitType.Angle); // Degrees

        public static final double REEF_APRILTAG_HEIGHT = 16; // Distance.ofBaseUnits(6.875, Inches);
        public static final double PROCCESSOR_APRILTAG_HEIGHT = 45.875; // Inches
        public static final double CORAL_APRILTAG_HEIGHT = 53.25; // Inches

        public static final edu.wpi.first.math.Vector<N3> MT1_STDEVS = VecBuilder.fill(0.5, 0.5, 1.0);
        public static final edu.wpi.first.math.Vector<N3> MT2_STDEVS = VecBuilder.fill(0.7, 0.7, 3492);
    }
}
