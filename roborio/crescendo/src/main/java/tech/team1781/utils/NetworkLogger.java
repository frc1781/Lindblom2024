package tech.team1781.utils;

import com.fasterxml.jackson.databind.jsontype.impl.TypeDeserializerBase;
import com.fasterxml.jackson.databind.util.ClassUtil;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import tech.team1781.ConfigMap;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.HashMap;

public class NetworkLogger {
    private static HashMap<String, GenericEntry> mEntryMap = new HashMap<String, GenericEntry>();

    public static void initLog(String name, Object defaultVal) {
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

    // private HashMap<String, GenericEntry> tabMap = new HashMap<String,
    // GenericEntry>();

    // public void log(String tabName, double data) {
    // GenericEntry tab = findTab(tabName, "Double");

    // tab.setDouble(new BigDecimal(data).setScale(2,
    // RoundingMode.HALF_UP).doubleValue());
    // }

    // public void log(String tabName, String data) {
    // GenericEntry tab = findTab(tabName, "String");
    // tab.setString(data);
    // }

    // private GenericEntry findTab(String tabName, String type) {
    // if (!tabMap.containsKey(tabName)) {
    // tabMap.put(tabName, ConfigMap.SHUFFLEBOARD_TAB.add(tabName, type == "Double"
    // ? 0 : "EMPTY" ).getEntry());
    // }

    // return tabMap.get(tabName);
    // }

}
