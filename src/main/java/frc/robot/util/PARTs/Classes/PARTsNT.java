package frc.robot.util.PARTs.Classes;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * PARTs NetworkTables Easy API.<p>
 * {@code PARTsNT partsNT = new PARTsNT(this);}<p>
 * This class is meant to be used as an instance for each class.
 */
public class PARTsNT {
    public String name = "Generic";

    NetworkTableInstance nt_Instance = NetworkTableInstance.getDefault();
    NetworkTable table;

    /**
     * Generic topic entry that is not designed to be used by itself.<p>
     * Use the other chilld classes instead.
     */
    private class EasyGenericEntry {
        /** The NetworkTables topic name. */
        public String topicName;

        public EasyGenericEntry() {
        }
    }

    class EasyBooleanEntry extends EasyGenericEntry {
        /** The NetworkTables topic name. */
        public String topicName;
        public BooleanTopic topic;
        public BooleanEntry entry;
        /** 
         * The value last gotten from set or get.<p>
         * Its purpose here is to provide a way to get the previous cached result, saving some time.
        */
        public boolean cachedValue;

        public EasyBooleanEntry(String name) {
            topicName = name;
            topic = table.getBooleanTopic(name);
            entry = topic.getEntry(false);
        }

        public EasyBooleanEntry(String name, Boolean value) {
            topicName = name;
            topic = table.getBooleanTopic(name);
            entry = topic.getEntry(false);
            entry.set(value);
            cachedValue = value;
        }
    }

    class EasyIntegerEntry extends EasyGenericEntry {
        /** The NetworkTables topic name. */
        public String topicName;
        public IntegerTopic topic;
        public IntegerEntry entry;
        /** 
         * The value last gotten from set or get.<p>
         * Its purpose here is to provide a way to get the previous cached result, saving some time.
        */
        public int cachedValue;

        public EasyIntegerEntry(String name) {
            topicName = name;
            topic = table.getIntegerTopic(name);
            entry = topic.getEntry(0);
        }

        public EasyIntegerEntry(String name, int value) {
            topicName = name;
            topic = table.getIntegerTopic(name);
            entry = topic.getEntry(0);
            entry.set(value);
            cachedValue = value;
        }
    }

    class EasyDoubleEntry extends EasyGenericEntry {
        /** The NetworkTables topic name. */
        public String topicName;
        public DoubleTopic topic;
        public DoubleEntry entry;
        /** 
         * The value last gotten from set or get.<p>
         * Its purpose here is to provide a way to get the previous cached result, saving some time.
        */
        public double cachedValue;

        public EasyDoubleEntry(String name) {
            topicName = name;
            topic = table.getDoubleTopic(name);
            entry = topic.getEntry(0.0);
        }

        public EasyDoubleEntry(String name, double value) {
            topicName = name;
            topic = table.getDoubleTopic(name);
            entry = topic.getEntry(0.0);
            entry.set(value);
            cachedValue = value;
        }
    }

    class EasyStringEntry extends EasyGenericEntry {
        /** The NetworkTables topic name. */
        public String topicName;
        public StringTopic topic;
        public StringEntry entry;
        /** 
         * The value last gotten from set or get.<p>
         * Its purpose here is to provide a way to get the previous cached result, saving some time.
        */
        public String cachedValue;

        public EasyStringEntry(String name) {
            topicName = name;
            topic = table.getStringTopic(name);
            entry = topic.getEntry("");
        }

        public EasyStringEntry(String name, String value) {
            topicName = name;
            topic = table.getStringTopic(name);
            entry = topic.getEntry("");
            entry.set(value);
            cachedValue = value;
        }
    }

    List<List> masterList;
    List<EasyGenericEntry> topicsList;
    List<EasyBooleanEntry> booleanEntries;
    List<EasyIntegerEntry> integerEntries;
    List<EasyDoubleEntry> doubleEntries;
    List<EasyStringEntry> stringEntries;

    /**
     * Sets up the master list and the entry lists.<p>
     * Internal function.
     */
    private void setupEntryLists() {
        topicsList = new ArrayList<>();
        masterList = new ArrayList<>();
        booleanEntries = new ArrayList<>();
        integerEntries = new ArrayList<>();
        doubleEntries = new ArrayList<>();
        stringEntries = new ArrayList<>();
        masterList.add(topicsList);
        masterList.add(booleanEntries);
        masterList.add(integerEntries);
        masterList.add(doubleEntries);
        masterList.add(stringEntries);
    }

    /**
     * Creates a new PARTsNT instance.<p>
     * Creates/uses the subtable "Generic" instead of the class subtable.<p>
     * The object variation should be used instead.
     */
    public PARTsNT() {
        table = nt_Instance.getTable("PARTs").getSubTable("Generic");
        setupEntryLists();
    }

    /**
     * Creates a new PARTsNT instance.<p>
     * Creates/uses the class subtable.
     * @param o The class object. (E.g. passing in 'this'.)
     */
    public PARTsNT(Object o) {
        name = o.getClass().getSimpleName();
        table = nt_Instance.getTable("PARTs").getSubTable(name);
        setupEntryLists();
    }

    /**
     * Creates a new PARTsNT instance.<p>
     * Creates/uses the subtable of the class via its name.<p>
     * If the name is empty, then the "Generic" table will be used instead.
     * @param className The name of the class.
     */
    public PARTsNT(String className) {
        name = (className != "") ? className : "Generic";
        table = nt_Instance.getTable("PARTs").getSubTable(name);
        setupEntryLists();
    }

    //* -------- HELPER FUNCTIONS -------- *//

    /**
     * Adds an entry to the list if the entry is not already on the list.
     * @param entry The entry to be added.
     */
    private void addEntryToList(EasyGenericEntry entry) {
        // Check if the entry already exists in the list.
        for (int i = 0; i < topicsList.size(); i++) {
            if (topicsList.get(i).topicName.equals(entry.topicName)) {
                return; // It exists so we abort adding it to avoid dupes.
            }
        }
        // Add the EasyGenericEntry. (Could also be any child.)
        topicsList.add(entry);
    }

    /**
     * Gets an existing entry from the entry list.
     * @param name The name of the entry.
     * @return The entry if it exists, otherwise null.
     */
    private EasyGenericEntry getEntry(String name) {
        for (EasyGenericEntry entry : topicsList) {
            if (entry.topicName.equals(name))
                return entry;
        }
        return null;
    }

    //* -------- TYPE SPECIFIC ENTRY CHECKS -------- *//

    /**
     * Gets an existing entry from the entry list.
     * @param name The name of the entry.
     * @return The entry if it exists, otherwise null.
     */
    private EasyBooleanEntry getBooleanEntry(String name) {
        for (EasyBooleanEntry entry : booleanEntries) {
            if (entry.topicName.equals(name))
                return entry;
        }
        return null;
    }

    /**
     * Gets an existing entry from the entry list.
     * @param name The name of the entry.
     * @return The entry if it exists, otherwise null.
     */
    private EasyIntegerEntry getIntegerEntry(String name) {
        for (EasyIntegerEntry entry : integerEntries) {
            if (entry.topicName.equals(name))
                return entry;
        }
        return null;
    }

    /**
     * Gets an existing entry from the entry list.
     * @param name The name of the entry.
     * @return The entry if it exists, otherwise null.
     */
    private EasyDoubleEntry getDoubleEntry(String name) {
        for (EasyDoubleEntry entry : doubleEntries) {
            if (entry.topicName.equals(name))
                return entry;
        }
        return null;
    }

    /**
     * Gets an existing entry from the entry list.
     * @param name The name of the entry.
     * @return The entry if it exists, otherwise null.
     */
    private EasyStringEntry getStringEntry(String name) {
        for (EasyStringEntry entry : stringEntries) {
            if (entry.topicName.equals(name))
                return entry;
        }
        return null;
    }

    //* -------- BOOLEAN FUNCTIONS -------- *//

    /**
     * Gets boolean value from the requested entry.
     * @param name The topic name.
     * @return Returns the boolean value if entry is found, otherwise returns false.
     */
    public boolean getBoolean(String name) {
        EasyBooleanEntry entry = getBooleanEntry(name);
        return (entry == null) ? false : (entry.cachedValue = entry.entry.get());
    }

    /**
     * Sets the boolean value for the requested entry.
     * @param name The name of the entry.
     * @param value The new value to publish to the entry.
     */
    public void putBoolean(String name, boolean value) {
        EasyBooleanEntry entry = getBooleanEntry(name);
        if (entry == null) {
            booleanEntries.add(new EasyBooleanEntry(name, value));
        } else if (entry.cachedValue != value) {
            entry.entry.set((entry.cachedValue = value));
        }
    }

    //* -------- INTEGER FUNCTIONS -------- *//

    /**
     * Gets the integer value from the requested entry.
     * @param name The name of the entry.
     * @return Returns the value if entry is found, otherwise returns zero.
     */
    public int getInteger(String name) {
        EasyIntegerEntry entry = getIntegerEntry(name);
        return (entry == null) ? 0 : (entry.cachedValue = Math.toIntExact(entry.entry.get()));
    }

    /**
     * Sets the integer value for the requested entry.
     * @param name The name of the entry.
     * @param value The new value to publish to the entry.
     */
    public void putInteger(String name, int value) {
        EasyIntegerEntry entry = getIntegerEntry(name);
        if (entry == null) {
            integerEntries.add(new EasyIntegerEntry(name, value));
        } else if (entry.cachedValue != value) {
            entry.entry.set((entry.cachedValue = value));
        }
    }

    //* -------- DOUBLE FUNCTIONS -------- *//

    /**
     * Gets the double value from the requested entry.
     * @param name The name of the entry.
     * @return Returns the value if entry is found, otherwise returns zero.
     */
    public double getDouble(String name) {
        EasyDoubleEntry entry = getDoubleEntry(name);
        return (entry == null) ? 0 : (entry.cachedValue = entry.entry.get());
    }

    /**
     * Sets the double value for the requested entry.
     * @param name The name of the entry.
     * @param value The new value to publish to the entry.
     */
    public void putDouble(String name, double value) {
        EasyDoubleEntry entry = getDoubleEntry(name);
        if (entry == null) {
            doubleEntries.add(new EasyDoubleEntry(name, value));
        } else if (entry.cachedValue != value) {
            entry.entry.set((entry.cachedValue = value));
        }
    }

    /**
     * Sets the double value for the requested entry.
     * @param name The name of the entry.
     * @param value The new value to publish to the entry.
     */
    public void putNumber(String name, double value) {
        putDouble(name, value);
    }

    /**
     * Sets the integer value for the requested entry.
     * @param name The name of the entry.
     * @param value The new value to publish to the entry.
     */
    public void putNumber(String name, int value) {
        putInteger(name, value);
    }

    //* -------- STRING FUNCTIONS -------- *//

    /**
     * Gets the string value from the requested entry.
     * @param name The name of the entry.
     * @return Returns the value if entry is found, otherwise returns an empty string.
     */
    public String getString(String name) {
        EasyStringEntry entry = getStringEntry(name);
        return (entry == null) ? "" : (entry.cachedValue = entry.entry.get());
    }

    /**
     * Sets the string value for the requested entry.
     * @param name The name of the entry.
     * @param value The new value to publish to the entry.
     */
    public void putString(String name, String value) {
        EasyStringEntry entry = getStringEntry(name);
        if (entry == null) {
            stringEntries.add(new EasyStringEntry(name, value));
        } else if (entry.cachedValue != value) {
            entry.entry.set((entry.cachedValue = value));
        }
    }

    //* -------- REMOVAL FUNCTIONS -------- *//

    /**
     * Removes all previously created entries.
     */
    public void removeAllEntries() {
        for (int i = 0; i < masterList.size(); i++) {
            for (int j = 0; j < masterList.get(i).size(); j++) {
                masterList.get(i).set(j, null);
                masterList.get(i).remove(j);
            }
        }
    }

    /**
     * Removes a previously created entry.
     * @param name The name of the entry to remove.
     */
    public void removeEntry(String name) {
        for (int i = 0; i < masterList.size(); i++) {
            for (int j = 0; j < masterList.get(i).size(); j++) {
                if (((EasyGenericEntry) masterList.get(i).get(j)).topicName.equals(name)) {
                    masterList.get(i).set(j, null);
                    masterList.get(i).remove(j);
                }
            }
        }
    }

    /**
     * Adds a sendable to smart dashboard network table entry.
     * @param data The sendable to add.
     */
    public void putSmartDashboardSendable(String key, Sendable data) {
        SmartDashboard.putData(String.format("%s/%s", name, key), data);
    }
}