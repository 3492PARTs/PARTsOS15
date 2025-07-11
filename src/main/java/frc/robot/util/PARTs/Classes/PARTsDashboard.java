package frc.robot.util.PARTs.Classes;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.Elastic;
import frc.robot.util.PARTs.Interfaces.IPARTsSubsystem;

public class PARTsDashboard {
    private static DashboardTab state = DashboardTab.AUTONOMOUS;

    public enum DashboardTab {
        AUTONOMOUS("Autonomous"),
        TELEOPERATED("Teleoperated"),
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
