package frc.lib.PARTsLib.CheckPARTs;

import java.util.ArrayList;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.PARTsLib.PARTsLibConstants;
import frc.lib.PARTsLib.CheckPARTs.PARTsError.PartStatus;
import frc.robot.Robot;

public class CheckPARTs {
    public static CheckPARTs instance;
    protected static ArrayList<PARTsError> errorLog;

    public static CheckPARTs getInstance() {
        if (instance == null) instance = new CheckPARTs();
        return instance;
    }

    ArrayList<ICheckPARTs> partsList;

    public CheckPARTs() {
        partsList = new ArrayList<>();
        errorLog = new ArrayList<>();
    }

    public void getReport(PARTsError report) {
        errorLog.add(report);
        if (report.currentStatus == PartStatus.OK) { System.out.print(report.message); return; }
        if (PARTsLibConstants.CheckPARTsConstants.SEND_TO_DASHBOARD && report.partsNT != null) {
            report.partsNT.sendNotificationToElasticDashboard(report.getNotificationLevel(), report.message);
        }
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().disable();
        System.err.println("PARTs Checker: Error in subsystem!\nThe scope: " + report.name + "\nThe Message: " + report.message + (PARTsLibConstants.CheckPARTsConstants.QUIT_ROBOT_CODE ? "\nThe robot code will now exit." : ""));
        if (PARTsLibConstants.CheckPARTsConstants.QUIT_ROBOT_CODE) {
            HAL.terminate();
            HAL.shutdown();
            System.exit(1);
        }
    }
}
