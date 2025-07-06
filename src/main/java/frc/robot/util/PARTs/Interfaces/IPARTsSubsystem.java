package frc.robot.util.PARTs.Interfaces;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IPARTsSubsystem extends Subsystem, Sendable {

    public void outputTelemetry();

    public void stop();

    public void reset();

    public void log();
}
