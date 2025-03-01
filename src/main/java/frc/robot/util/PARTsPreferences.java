package frc.robot.util;

import java.lang.reflect.Type;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.Preferences;

/**
 * Manager class for each {@link PARTsPreference}.
 */
public class PARTsPreferences {

    private enum PrefType {
        Boolean(),
        Int(),
        Double(),
        Float(),
        String();
    };

    /**
     * Prefrence object that gets assigned to a variable.<p>
     * This class is designed to work with its parent, PARTsPrefrences.<p>
     * It holds the key and type and returns and sets the prefrence value.
     */
    public class PARTsPreference {
        public String key;
        public Type type;
        private PrefType _type;

        public PARTsPreference(String key, Boolean value) {
            this.key = key;
            Preferences.initBoolean(key, value);
            _type = PrefType.Boolean;
            type = value.getClass();
        }

        public PARTsPreference(String key, Integer value) {
            this.key = key;
            Preferences.initInt(key, value);
            _type = PrefType.Boolean;
            type = value.getClass();
        }
        
        public PARTsPreference(String key, Double value) {
            this.key = key;
            Preferences.initDouble(key, value);
            _type = PrefType.Double;
            type = value.getClass();
        }

        public PARTsPreference(String key, Float value) {
            this.key = key;
            Preferences.initFloat(key, value);
            _type = PrefType.Float;
            type = value.getClass();
        }

        public PARTsPreference(String key, String value) {
            this.key = key;
            Preferences.initString(key, value);
            _type = PrefType.String;
            type = value.getClass();
        }

        public String getKey() {
            return key;
        }

        public Type getType() {
            return type;
        }

        public Object getValue() {
            switch (_type) {
                case Boolean: return Preferences.getBoolean(key, false);
                case Int: return Preferences.getInt(key, 0);
                case Double: return Preferences.getDouble(key, 0);
                case Float: return Preferences.getFloat(key, 0);
                case String: return Preferences.getString(key, "");
                default: throw new TypeNotPresentException(_type.name(), new Throwable("Type is an invalid Prefrence type!"));
            }
        }

        public void setValue(Object value) {
            switch (_type) { 
                case Boolean: Preferences.setBoolean(key, ((Boolean) value));
                case Int: Preferences.setInt(key, ((Integer) value));
                case Double: Preferences.setDouble(key, ((Double) value));
                case Float: Preferences.setFloat(key, ((Float) value));
                case String: Preferences.setString(key, ((String) value));
                default: throw new TypeNotPresentException(_type.name(), new Throwable("Type is an invalid Prefrence type!"));
            }
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
