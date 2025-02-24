package frc.robot.util;

import java.util.ArrayList;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PARTsDashboard {
    private static DashboardTab state = DashboardTab.AUTONOMOUS;

    public enum DashboardTab {
        AUTONOMOUS("Autonomous"),
        TEHEOPERATED("Teleoperated"),
        DEBUG("Dashboard");

        String tabName;

        DashboardTab(String s) {
            this.tabName = s;
        }
    }

    public PARTsDashboard() {
    }

    public static void setSubsystems(ArrayList<IPARTsSubsystem> subsystems) {
        subsystems.forEach(s -> SmartDashboard.putData(s));
    }

    public static void setCommandScheduler() {
        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    public static void setTab(DashboardTab dashboardState) {
        state = dashboardState;
        Elastic.selectTab(state.tabName);
    }
}
