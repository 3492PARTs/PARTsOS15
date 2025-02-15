package frc.robot.util;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants;

public class PARTsLogger {
    private static DataLog log;

    public PARTsLogger() {
        if (Constants.Debug.logging) {
            // Starts recording to data log
            DataLogManager.start();

            log = DataLogManager.getLog();
            // Record both DS control and joystick data
            //DriverStation.startDataLog(log);
        }
    }

    public boolean logBoolean(String key, boolean b) {
        if (Constants.Debug.logging) {
            new BooleanLogEntry(log, key).append(b);
            return true;
        } else
            return false;
    }

    public boolean logDouble(String key, double d) {
        if (Constants.Debug.logging) {
            new DoubleLogEntry(log, key).append(d);
            return true;
        } else
            return false;
    }

    public boolean logString(String key, String s) {
        if (Constants.Debug.logging) {
            new StringLogEntry(log, key).append(s);
            return true;
        } else
            return false;
    }
}
