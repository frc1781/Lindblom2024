package tech.team1781.utils;

import edu.wpi.first.networktables.GenericEntry;
import tech.team1781.ConfigMap;

import java.util.HashMap;

public class NetworkLogger {
    private static HashMap<String, GenericEntry> mEntryMap = new HashMap<String, GenericEntry>();

    public static void initLog(String name, Object defaultVal) {
        if(mEntryMap.containsKey(name)){
            return;
        }
        mEntryMap.put(
                name,
                ConfigMap.LOG_TAB.add(
                    name, 
                    isPrimitive(defaultVal) ? defaultVal : defaultVal.toString()
                ).getEntry());
    }

    public static void logData(String logName, Object val) {
        var entry = mEntryMap.get(logName);

        if (entry == null) {
            return;
        }

        entry.setValue(isPrimitive(val) ? val : val.toString());
    }

    public static boolean isPrimitive(Object obj) {
        switch (obj.getClass().getName()) {
            case "java.lang.Integer":
            case "java.lang.Double":
            case "java.lang.Float":
            case "java.lang.Long":
            case "java.lang.Short":
            case "java.lang.Byte":
            case "java.lang.Boolean":
            case "java.lang.Character":
            case "java.lang.String":
                return true;
            default:
                return false;
        }
    }
}
