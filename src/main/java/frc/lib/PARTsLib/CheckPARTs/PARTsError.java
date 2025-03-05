package frc.lib.PARTsLib.CheckPARTs;

public class PARTsError {
    public enum PartStatus {
        OK(),
        UNKNOWN(),
        SENSOR_FAILURE(),
        MOTOR_FAILURE(),
        CODE_FAILURE(),
        FAILURE();
    };

    PartStatus currentStatus;
    String message;
    String name;

    public PARTsError(String name) {
        currentStatus = PartStatus.UNKNOWN;
        message = "";
        this.name = name;
    }

    public PARTsError(String name, PartStatus status) {
        currentStatus = status;
        message = "";
        this.name = name;
    }

    public PARTsError(String name, PartStatus status, String message) {
        currentStatus = status;
        this.message = message;
        this.name = name;
    }

    public PartStatus getStatus() {
        return currentStatus;
    }
}
