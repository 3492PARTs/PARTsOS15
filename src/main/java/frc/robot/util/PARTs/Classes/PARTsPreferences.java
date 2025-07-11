package frc.robot.util.PARTs.Classes;

import java.lang.reflect.Type;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.Preferences;

/**
 * Manager class for each {@link PARTsPreference}.
 */
public class PARTsPreferences {
    /**
     * Prefrence object that gets assigned to a variable.<p>
     * This class is designed to work with its parent, PARTsPrefrences.<p>
     * It holds the key and type and returns and sets the prefrence value.
     */
    public class PARTsPreference {
        public String key;
        public Type type;

        public PARTsPreference(String key, Boolean value) {
            this.key = key;
            Preferences.initBoolean(key, value);
            type = value.getClass();
        }

        public PARTsPreference(String key, Integer value) {
            this.key = key;
            Preferences.initInt(key, value);
            type = value.getClass();
        }
        
        public PARTsPreference(String key, Double value) {
            this.key = key;
            Preferences.initDouble(key, value);
            type = value.getClass();
        }

        public PARTsPreference(String key, Float value) {
            this.key = key;
            Preferences.initFloat(key, value);
            type = value.getClass();
        }

        public PARTsPreference(String key, String value) {
            this.key = key;
            Preferences.initString(key, value);
            type = value.getClass();
        }

        public String getKey() {
            return key;
        }

        public Type getType() {
            return type;
        }

        public Boolean getBoolean() {
            return Preferences.getBoolean(key, false);
        }

        public Integer getInteger() {
            return Preferences.getInt(key, 0);
        }

        public Double getDouble() {
            return Preferences.getDouble(key, 0);
        }

        public Float getFloat() {
            return Preferences.getFloat(key, 0);
        }

        public String getString() {
            return Preferences.getString(key, key);
        }

        /* --- Setters --- */

        public void setBoolean(Boolean value) {
            Preferences.setBoolean(key, value);
        }

        public void setBoolean(Integer value) {
            Preferences.setInt(key, value);
        }

        public void setBoolean(Double value) {
            Preferences.setDouble(key, value);
        }

        public void setBoolean(Float value) {
            Preferences.setFloat(key, value);
        }

        public void setBoolean(String value) {
            Preferences.setString(key, value);
        }
    }

    public ArrayList<PARTsPreference> prefrences;

    public PARTsPreferences() {
        prefrences = new ArrayList<>();
    }

    public PARTsPreference addPreference(String name, Boolean value) {
        PARTsPreference pref = new PARTsPreference(name, value);
        prefrences.add(pref);
        return pref;
    }

    public PARTsPreference addPreference(String name, Integer value) {
        PARTsPreference pref = new PARTsPreference(name, value);
        prefrences.add(pref);
        return pref;
    }

    public PARTsPreference addPreference(String name, Double value) {
        PARTsPreference pref = new PARTsPreference(name, value);
        prefrences.add(pref);
        return pref;
    }

    public PARTsPreference addPreference(String name, Float value) {
        PARTsPreference pref = new PARTsPreference(name, value);
        prefrences.add(pref);
        return pref;
    }

    public PARTsPreference addPreference(String name, String value) {
        PARTsPreference pref = new PARTsPreference(name, value);
        prefrences.add(pref);
        return pref;
    }
}
