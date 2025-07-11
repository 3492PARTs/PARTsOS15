package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.PARTs.Classes.PARTsUnit;
import frc.robot.util.PARTs.Classes.PARTsUnit.PARTsUnitType;

public class AlgaeConstants {
    public static final int algaeIntakeId = 3;
    public static final int algaeWristId = 4;
    public static final double algaeIntakeSpeed = 0.2;
    public static final double maxWristSpeed = .5;
    public static final int kMaxWristCurrent = 10;

    public static final double kWristP = 10.0;
    public static final double kWristI = 0.0;
    public static final double kWristD = 0.0;
    public static final double kTolerance = new PARTsUnit(1, PARTsUnitType.Angle).to(PARTsUnitType.Radian);

    public static final double kWristKS = 0.25008;
    public static final double kWristKG = 0.10667;
    public static final double kWristKV = 0.0019276;
    public static final double kWristKA = 0.00032922;

    public static final double kWristOffset = 0;// 141.0;

    public static final double kWristMaxVelocity = 690.0;
    public static final double kWristMaxAcceleration = 1380.0;

    public static final double kStowAngle = 0;
    public static final double kDeAlgaeAngle = 139.15;
    public static final double kGroundIntakeAngle = 145.0;

    public static final int wristGearRatio = 64 / 1;

    // INTAKE
    public static final int kMaxIntakeCurrent = 20;

    public static final double kReefIntakeSpeed = 0.8;
    public static final double kEjectSpeed = 0.3;
    public static final double kGroundIntakeSpeed = -0.3;

    public static final Pose2d deAlgaeDistance = new Pose2d(0.5, 0, new Rotation2d());
}
