package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.PARTs.Classes.PARTsUnit;
import frc.robot.util.PARTs.Classes.PARTsUnit.PARTsUnitType;

public class AlgaeConstants {
    public static final int INTAKE_CAN_ID = 3;
    public static final int WRIST_CAN_ID = 4;
    public static final double ALGAE_INTAKE_SPEED = 0.2;
    public static final double MAX_WRIST_SPEED = .5;
    public static final int MAX_WRIST_CURRENT = 10;

    public static final double WRIST_P = 10.0;
    public static final double WRIST_I = 0.0;
    public static final double WRIST_D = 0.0;
    public static final double WRIST_TOLERANCE = new PARTsUnit(1, PARTsUnitType.Angle).to(PARTsUnitType.Radian);

    public static final double WRIST_KS = 0.25008;
    public static final double WRIST_KG = 0.10667;
    public static final double WRIST_KV = 0.0019276;
    public static final double WRIST_KA = 0.00032922;

    public static final double WRIST_OFFSET = 0;// 141.0;

    public static final double WRIST_MAX_VELOCITY = 690.0;
    public static final double WRIST_MAX_ACCELERATION = 1380.0;

    public static final double STOW_ANGLE = 0;
    public static final double DEALGAE_ANGLE = 139.15;
    public static final double GROUND_INTAKE_ANGLE = 145.0;

    public static final int WRIST_GEAR_RATIO = 64 / 1;

    // INTAKE
    public static final int MAX_INTAKE_CURRENT = 20;

    public static final double REEF_INTAKE_SPEED = 0.8;
    public static final double EJECT_SPEED = 0.3;
    public static final double GROUND_INTAKE_SPEED = -0.3;

    public static final Pose2d DEALGAE_DISTANCE = new Pose2d(0.5, 0, new Rotation2d());
}
