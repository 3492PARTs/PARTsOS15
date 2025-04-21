package frc.lib.PARTsLib.CheckPARTs;

import java.util.Date;

import frc.lib.ElasticLib.Elastic.Notification.NotificationLevel;
import frc.lib.PARTsLib.PARTsLogger;
import frc.lib.PARTsLib.PARTsNT;

public class PARTsError {
    public enum PartStatus {
        OK(),
        UNKNOWN(),
        WARNING(),
        SENSOR_FAILURE(),
        MOTOR_FAILURE(),
        CODE_FAILURE(),
        FAILURE();
    };

    PartStatus currentStatus;
    String message;
    String name;
    PARTsNT partsNT;
    Date timestamp;

    public NotificationLevel getNotificationLevel(PartStatus status) {
        switch (status) {
            case CODE_FAILURE:
                return NotificationLevel.ERROR;
            case FAILURE:
                return NotificationLevel.ERROR;
            case MOTOR_FAILURE:
                return NotificationLevel.ERROR;
            case OK:
                return NotificationLevel.INFO;
            case WARNING:
                return NotificationLevel.WARNING;
            case SENSOR_FAILURE:
                return NotificationLevel.ERROR;
            case UNKNOWN:
                return NotificationLevel.INFO;
            default:
                return NotificationLevel.INFO;
        }
    }

    public NotificationLevel getNotificationLevel() {
        switch (currentStatus) {
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
        timestamp = new Date();
        currentStatus = PartStatus.UNKNOWN;
        message = "";
        this.name = name;
        partsNT.sendNotificationToElasticDashboard(NotificationLevel.ERROR, name);
    }

    public PARTsError(String name, PARTsLogger logger) {
        timestamp = new Date();
        currentStatus = PartStatus.UNKNOWN;
        message = "";
        this.name = name;
    }

    public PARTsError(String name, PartStatus status) {
        timestamp = new Date();
        currentStatus = status;
        message = "";
        this.name = name;
    }

    public PARTsError(String name, PartStatus status, PARTsLogger logger) {
        timestamp = new Date();
        currentStatus = status;
        message = "";
        this.name = name;
    }

    public PARTsError(String name, PartStatus status, PARTsNT partsNT) {
        timestamp = new Date();
        currentStatus = status;
        message = "";
        this.name = name;
        this.partsNT = partsNT;
    }

    public PARTsError(String name, PartStatus status, PARTsLogger logger, PARTsNT partsNT) {
        timestamp = new Date();
        currentStatus = status;
        message = "";
        this.name = name;
        this.partsNT = partsNT;
    }

    public PARTsError(String name, PartStatus status, String message) {
        timestamp = new Date();
        currentStatus = status;
        this.message = message;
        this.name = name;
    }

    public PARTsError(String name, PartStatus status, String message, PARTsLogger logger) {
        timestamp = new Date();
        currentStatus = status;
        this.message = message;
        this.name = name;
    }

    public PARTsError(String name, PartStatus status, String message, PARTsNT partsNT) {
        timestamp = new Date();
        currentStatus = status;
        this.message = message;
        this.name = name;
        this.partsNT = partsNT;
    }

    public PARTsError(String name, PartStatus status, String message, PARTsLogger logger, PARTsNT partsNT) {
        timestamp = new Date();
        currentStatus = status;
        this.message = message;
        this.name = name;
        this.partsNT = partsNT;
    }

    public PartStatus getStatus() {
        return currentStatus;
    }

    public void setStatus(PartStatus status) {
        currentStatus = status;
    }

    public void setMessage(String msg) {
        message = msg;
    }
}
