package frc.robot.util.PARTs.Classes;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.RobotConstants;
import frc.robot.util.Field.Field;

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
        if (RobotConstants.logging) {
            // Starts recording to data log
            DataLogManager.start();

            if (log == null)
                log = DataLogManager.getLog();
        }
    }

    public boolean logBoolean(String key, boolean b) {
        if (RobotConstants.logging) {
            new BooleanLogEntry(log, name.length() > 0 ? String.format("%s/%s", name, key) : key).append(b);
            return true;
        } else
            return false;
    }

    public boolean logDouble(String key, double d) {
        if (RobotConstants.logging) {
            new DoubleLogEntry(log, name.length() > 0 ? String.format("%s/%s", name, key) : key).append(d);
            return true;
        } else
            return false;
    }

    public boolean logString(String key, String s) {
        if (RobotConstants.logging) {
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

    public void logPathPlanner() {
        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            Field.FIELD2D.getObject("target pose").setPose(Field.conditionallyTransformToOppositeAlliance(pose));
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            Field.FIELD2D.getObject("path").setPoses(Field.conditionallyTransformToOppositeAlliance(poses));
        });
    }
}
