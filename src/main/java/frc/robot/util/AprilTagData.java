package frc.robot.util;

import frc.robot.util.PARTsUnit.PARTsUnitType;

public final class AprilTagData {
    public static enum AprilTagType {
        NONE(0),
        REEF(1),
        CAGE(2),
        PROCESSOR(3),
        STATION(4);
    
        public final int index;
    
        private AprilTagType(int index) {
            this.index = index;
        }
    }

    static final AprilTagType[] APRIL_TAG_TYPES = {
        AprilTagType.NONE,      AprilTagType.STATION,   AprilTagType.STATION,
        AprilTagType.PROCESSOR, AprilTagType.CAGE,      AprilTagType.CAGE,
        AprilTagType.REEF,      AprilTagType.REEF,      AprilTagType.REEF,
        AprilTagType.REEF,      AprilTagType.REEF,      AprilTagType.REEF,
        AprilTagType.STATION,   AprilTagType.STATION,   AprilTagType.CAGE,
        AprilTagType.CAGE,      AprilTagType.PROCESSOR, AprilTagType.REEF,
        AprilTagType.REEF,      AprilTagType.REEF,      AprilTagType.REEF,
        AprilTagType.REEF
    };

    public static class AprilTagObject {
        public int tagID = 0;
        public PARTsUnit angle = new PARTsUnit(0, PARTsUnitType.Angle);
        PARTsUnit height = new PARTsUnit(0, null);
        AprilTagType type = AprilTagType.NONE;

        /**
         * Create a new AprilTag object.
         * @param height Height in inches.
         */
        AprilTagObject(double height, AprilTagType type, int knownID) {
            this.height = new PARTsUnit(height, PARTsUnitType.Inch);
            this.type = type;
            tagID = knownID;
        }

        /**
         * Create a new AprilTag object.
         * @param height Height in inches.
         */
        AprilTagObject(double height, AprilTagType type, int knownID, PARTsUnit angle) {
            this.height = new PARTsUnit(height, PARTsUnitType.Inch);
            this.type = type;
            tagID = knownID;
            this.angle = angle;
        }
    }

    static final AprilTagObject[] APRILTAG_TAG_OBJECTS = new AprilTagObject[22]; //= {new AprilTagObject(16, AprilTagType.NONE, 0)};

    static double centerOffset = 3.25;
    public static final void InitAprilTagObjects() {
        for (int i=0; i < APRIL_TAG_TYPES.length; i++) {
            double currHeight = 0.0;
            int angle = 0;
            switch (APRIL_TAG_TYPES[i]) {
                case CAGE:
                    currHeight = 70.73;
                    break;
                case NONE:
                    currHeight = 16.0-centerOffset;
                    break;
                case PROCESSOR:
                    currHeight = 47.88;
                    break;
                case REEF:
                    currHeight = 8.75;
                    if (i == 18 || i == 10) angle = 135;
                    if (i == 17 || i == 11) angle = 135+45;
                    if (i == 22 || i == 6) angle = 135+90;
                    if (i == 21 || i == 7) angle = 135*2;
                    if (i == 20 || i == 8) angle = 135-45;
                    if (i == 19 || i == 9) angle = 135-90;
                    break;
                case STATION:
                    currHeight = 55.25;
                    break;
                default:
                    throw new RuntimeException("AprilTag type is not a valid type!");
            }



            APRILTAG_TAG_OBJECTS[i] = new AprilTagObject(
                currHeight + centerOffset, 
                APRIL_TAG_TYPES[i], 
                i,
                new PARTsUnit(angle, PARTsUnitType.Angle)
            );
        }
    }

    // Ignores None and proccessors by default.
    public static int[] customIgnoreIDList = {0,3,16};

    private static boolean isValidID(int targetID) {
        for (int elem : customIgnoreIDList) {
            if (elem == targetID) return false;
        }

        for (AprilTagObject elem : APRILTAG_TAG_OBJECTS) {
            if (elem.tagID == targetID) return true;
        }
        return false;
    }

    /**
     * Gets requested target AprilTag type from the requested AprilTag.
     * <p>
     * Returns {@link AprilTagType NONE} if the ID is not a valid field ID and the ID is in the ignore list.
     * @param targetID The ID of the requested target.
     * @return The {@link AprilTagType AprilTagType} of the target AprilTag.
     */
    public static AprilTagType getTargeTagType(int targetID) {
        return (targetID > 0 && targetID <= 22 && isValidID(targetID)) ? APRIL_TAG_TYPES[targetID] : AprilTagType.NONE;
    }

    /**
     * Gets requested target AprilTag type from the requested AprilTag.
     * <p>
     * Returns {@link AprilTagType NONE} if the ID is not a valid field ID.
     * @param targetID The ID of the requested target.
     * @return The {@link AprilTagType AprilTagType} of the target AprilTag.
     */
    public static AprilTagType getTargeTagType(int targetID, boolean overrideIgnoreList) {
        return (targetID > 0 && targetID <= 22) ? APRIL_TAG_TYPES[targetID] : AprilTagType.NONE;
    }

    /**
     * Gets the height of the AprilTag as a {@link frc.robot.util.PARTsUnit PARTsUnit}.
     * <p>
     * If the ID is not valid, a runtime exception will be thrown.
     * @param targetID The ID of the requested AprilTag.
     * @return The height of the AprilTag as a {@link frc.robot.util.PARTsUnit PARTsUnit}.
     */
    public static PARTsUnit getAprilTagHeight(int targetID) {
        return APRILTAG_TAG_OBJECTS[targetID].height;
    }

    public static PARTsUnit getAprilTagAngle(int targetID) {
        return APRILTAG_TAG_OBJECTS[targetID].angle;
    }
}
