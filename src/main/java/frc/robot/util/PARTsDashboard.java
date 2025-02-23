package frc.robot.util;

import au.grapplerobotics.LaserCan;

public class PARTsDashboard {
    private static DashboardState state = DashboardState.AUTONOMOUS;

    public enum DashboardState {
        AUTONOMOUS("Autonomous"),
        TEHEOPERATED("Teleoperated"),
        DEBUG("Dashboard");

        String tabName;

        DashboardState(String s) {
            this.tabName = s;
        }
    }

    public PARTsDashboard() {
    }

    public static void setDashboard(DashboardState dashboardState) {
        state = dashboardState;
        Elastic.selectTab(state.tabName);
    }
}
