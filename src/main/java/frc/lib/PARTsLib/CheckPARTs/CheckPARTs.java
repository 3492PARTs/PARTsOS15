package frc.lib.PARTsLib.CheckPARTs;

import java.util.ArrayList;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.PARTsLib.CheckPARTs.PARTsError.PartStatus;

public class CheckPARTs {
    public static CheckPARTs instance;

    public static CheckPARTs getInstance() {
        if (instance == null) instance = new CheckPARTs();
        return instance;
    }

    ArrayList<ICheckPARTs> partsList;

    public CheckPARTs() {
        partsList = new ArrayList<>();
    }

    public void getReport(PARTsError report) {
        if (report.currentStatus == PartStatus.OK) { System.out.print(report); return; }
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().disable();
        System.err.println("PARTs Checker: Error in subsystem!\nThe subsystem: " + report.name + "\nThe Message: " + report.message + "\nThe program will now exit.");
        System.exit(1);
    }

    /**
     * @deprecated Non-functional method.
     * @return
     */
    public ArrayList<PARTsError> testUnits() {
        ArrayList<PARTsError> results = new ArrayList<>();

        for (ICheckPARTs part : partsList) {
        }
        
        return results;
    }
}
