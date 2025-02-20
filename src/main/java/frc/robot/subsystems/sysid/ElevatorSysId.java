// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.sysid;

import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Elevator;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

/** Add your docs here. */
public class ElevatorSysId extends Elevator {
    private MutVoltage appliedVoltage;

    private MutDistance elevatorPosition;

    private MutLinearVelocity elevatorVelocity;

    private SysIdRoutine routine;

    public ElevatorSysId() {
        super();

        appliedVoltage = Volts.mutable(0);

        elevatorPosition = Inches.mutable(0);

        elevatorVelocity = InchesPerSecond.mutable(0);

        routine = new SysIdRoutine(
            new SysIdRoutine.Config(),//ElevatorConstants.kSysIDConfig,
            new SysIdRoutine.Mechanism(
                super.mLeftMotor::setVoltage,
                (log) -> {
                    log.motor("elevatorMotor1")
                        .voltage(appliedVoltage.mut_replace(
                            super.mLeftMotor.get() * RobotController.getBatteryVoltage(), Volts
                        ))
                        .linearPosition(elevatorPosition.mut_replace(
                            super.mLeftEncoder.getPosition(), Inches
                        ))
                        .linearVelocity(elevatorVelocity.mut_replace(
                            super.mLeftEncoder.getVelocity(), InchesPerSecond
                        ));
                }, 
                this
            )
        );
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
