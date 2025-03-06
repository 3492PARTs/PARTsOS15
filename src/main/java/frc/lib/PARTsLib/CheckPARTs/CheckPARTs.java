package frc.lib.PARTsLib.CheckPARTs;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.PARTsLib.PARTsLibConstants;
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
        if (report.currentStatus == PartStatus.OK) { System.out.print(report.message); return; }
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().disable();
        System.err.println("PARTs Checker: Error in subsystem!\nThe scope: " + report.name + "\nThe Message: " + report.message + (PARTsLibConstants.CheckPARTsConstants.QUIT_ROBOT_CODE ? "\nThe robot code will now exit." : ""));
        if (PARTsLibConstants.CheckPARTsConstants.QUIT_ROBOT_CODE)
            System.exit(1);
    }
}
