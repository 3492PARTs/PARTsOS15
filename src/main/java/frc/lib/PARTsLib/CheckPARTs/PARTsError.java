package frc.lib.PARTsLib.CheckPARTs;

import frc.lib.ElasticLib.Elastic.Notification.NotificationLevel;
import frc.lib.PARTsLib.PARTsLogger;
import frc.lib.PARTsLib.PARTsNT;

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

    private NotificationLevel getNotificationLevel(PartStatus status) {
        switch (status) {
            case CODE_FAILURE:
                return NotificationLevel.ERROR;
            case FAILURE:
                return NotificationLevel.ERROR;
            case MOTOR_FAILURE:
                return NotificationLevel.ERROR;
            case OK:
                return NotificationLevel.INFO;
            case SENSOR_FAILURE:
                return NotificationLevel.ERROR;
            case UNKNOWN:
                return NotificationLevel.INFO;
            default:
                return NotificationLevel.INFO;
        }
    }

    public PARTsError(String name, PARTsNT partsNT) {
        currentStatus = PartStatus.UNKNOWN;
        message = "";
        this.name = name;
        partsNT.sendNotificationToElasticDashboard(NotificationLevel.ERROR, name);
    }

    public PARTsError(String name, PARTsLogger logger) {
        currentStatus = PartStatus.UNKNOWN;
        message = "";
        this.name = name;
    }

    public PARTsError(String name, PartStatus status) {
        currentStatus = status;
        message = "";
        this.name = name;
    }

    public PARTsError(String name, PartStatus status, PARTsLogger logger) {
        currentStatus = status;
        message = "";
        this.name = name;
    }

    public PARTsError(String name, PartStatus status, PARTsNT partsNT) {
        currentStatus = status;
        message = "";
        this.name = name;
    }

    public PARTsError(String name, PartStatus status, PARTsLogger logger, PARTsNT partsNT) {
        currentStatus = status;
        message = "";
        this.name = name;
    }

    public PARTsError(String name, PartStatus status, String message) {
        currentStatus = status;
        this.message = message;
        this.name = name;
    }

    public PARTsError(String name, PartStatus status, String message, PARTsLogger logger) {
        currentStatus = status;
        this.message = message;
        this.name = name;
    }

    public PARTsError(String name, PartStatus status, String message, PARTsNT partsNT) {
        currentStatus = status;
        this.message = message;
        this.name = name;
    }

    public PARTsError(String name, PartStatus status, String message, PARTsLogger logger, PARTsNT partsNT) {
        currentStatus = status;
        this.message = message;
        this.name = name;
    }

    public PartStatus getStatus() {
        return currentStatus;
    }
}
