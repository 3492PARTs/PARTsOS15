package frc.lib.PARTsLib;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.lib.PARTsLib.CheckPARTs.ICheckPARTs;
import frc.lib.PARTsLib.CheckPARTs.PARTsError;

public interface IPARTsSubsystem extends Subsystem, Sendable, ICheckPARTs {

    public void outputTelemetry();

    public void stop();

    public void reset();

    public void report(PARTsError error);

    public void log();
}
