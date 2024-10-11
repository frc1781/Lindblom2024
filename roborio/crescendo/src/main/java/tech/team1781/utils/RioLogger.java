package tech.team1781.utils;

import edu.wpi.first.util.datalog.*;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.HashMap;

public class RioLogger {
    private static HashMap<String, DataLogEntry> mEntries = new HashMap<>();

    public static void initLog(String name, Object defaultValue) {
        if (mEntries.containsKey(name)) {
            DriverStation.reportError("Entry " + name + " already exists", false);
            return;
        } else {
            mEntries.put(name, getEntry(name, defaultValue));
        }
    }

    public static void logData(String name, Object value) {
        if (mEntries.containsKey(name)) {
            switch(value.getClass().getName()) {
                case "java.lang.Integer":
                    ((IntegerLogEntry) mEntries.get(name)).append((int) value);
                    break;
                case "java.lang.Double":
                    ((DoubleLogEntry) mEntries.get(name)).append((double) value);
                    break;
                case "java.lang.Float":
                    ((FloatLogEntry) mEntries.get(name)).append((float) value);
                    break;
                case "java.lang.Boolean":
                    ((BooleanLogEntry) mEntries.get(name)).append((boolean) value);
                    break;
                default:
                    ((StringLogEntry) mEntries.get(name)).append(value.toString());
                    break;
            }
        }    
    }

    private static DataLogEntry getEntry(String name, Object defaultValue) {
        switch (defaultValue.getClass().getName()) {
            case "java.lang.Integer":
                return new IntegerLogEntry(DataLogManager.getLog(), "Logs/" + name);
            case "java.lang.Double":
                return new DoubleLogEntry(DataLogManager.getLog(), "Logs/" + name);
            case "java.lang.Float":
                return new FloatLogEntry(DataLogManager.getLog(), "Logs/" + name);
            case "java.lang.Boolean":
                return new BooleanLogEntry(DataLogManager.getLog(), "Logs/" + name);
            default:
                return new StringLogEntry(DataLogManager.getLog(), "Logs/" + name);

        }
    }

}
