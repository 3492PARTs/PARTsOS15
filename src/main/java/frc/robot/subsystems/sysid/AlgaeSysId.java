package frc.robot.subsystems.sysid;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.subsystems.Algae;

public class AlgaeSysId extends Algae {
    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutVoltage m_appliedVoltage = Volts.mutable(0);
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutAngle m_angle = Rotations.mutable(0);
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutAngularVelocity m_velocity = RotationsPerSecond.mutable(0);
    private SysIdRoutine routine;

    public AlgaeSysId() {
        routine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        super.mWristMotor::setVoltage,

                        log -> {
                            // Record a frame for the shooter motor.
                            log.motor("wrist")
                                    .voltage(
                                            m_appliedVoltage.mut_replace(
                                                    super.mWristMotor.getAppliedOutput() * mWristMotor.getBusVoltage(),
                                                    Volts))
                                    .angularPosition(m_angle.mut_replace(
                                            ((super.mWristRelEncoder.getPosition())),
                                            Rotations))
                                    .angularVelocity(
                                            m_velocity.mut_replace(getRPS(), RotationsPerSecond));
                        },
                        this));
    }

    @Override
    public void periodic() {
        //dummy to stop super periodic from running
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}
