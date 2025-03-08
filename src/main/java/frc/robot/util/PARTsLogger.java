package frc.robot.util;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;

public class PARTsLogger {
    private static DataLog log;
    private String name = "";

    public PARTsLogger() {
        instantiate();
    }

    public PARTsLogger(String name) {
        instantiate();
        this.name = name;
    }

    public PARTsLogger(Object o) {
        name = o.getClass().getSimpleName();
        instantiate();
    }

    private void instantiate() {
        if (Constants.Debug.logging) {
            // Starts recording to data log
            DataLogManager.start();

            if (log == null)
                log = DataLogManager.getLog();
        }
    }

    public boolean logBoolean(String key, boolean b) {
        if (Constants.Debug.logging) {
            new BooleanLogEntry(log, name.length() > 0 ? String.format("%s/%s", name, key) : key).append(b);
            return true;
        } else
            return false;
    }

    public boolean logDouble(String key, double d) {
        if (Constants.Debug.logging) {
            new DoubleLogEntry(log, name.length() > 0 ? String.format("%s/%s", name, key) : key).append(d);
            return true;
        } else
            return false;
    }

    public boolean logString(String key, String s) {
        if (Constants.Debug.logging) {
            new StringLogEntry(log, name.length() > 0 ? String.format("%s/%s", name, key) : key).append(s);
            return true;
        } else
            return false;
    }

    public void logCommandScheduler() {

        // Set the scheduler to log events for command initialize, interrupt, finish
        CommandScheduler.getInstance()
                .onCommandInitialize(
                        command -> {
                            logString(command.getName(), "Command initialized");
                        });
        CommandScheduler.getInstance()
                .onCommandInterrupt(
                        command -> {
                            logString(command.getName(), "Command interrupted");
                        });
        CommandScheduler.getInstance()
                .onCommandFinish(
                        command -> {
                            logString(command.getName(), "Command finished");
                        });
    }
}
