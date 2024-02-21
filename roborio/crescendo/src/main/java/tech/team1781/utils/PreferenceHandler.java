package tech.team1781.utils;

import java.util.HashMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Preferences;
import tech.team1781.ConfigMap;

public class PreferenceHandler {

    private static HashMap<String, ValueHolder<?>> valueHolders = new HashMap<>();

    public static double getDouble(String key, double defaultValue) {
        return Preferences.getDouble(key, defaultValue);
    }

    public static <T> void addValue(String key, T defaultValue) {
        ValueHolder<T> holder = new ValueHolder<>();
        holder.key = key;
        holder.defaultValue = defaultValue;
        holder.entry = ConfigMap.CONFIG_TAB.add(key, defaultValue).getEntry();

        if (!Preferences.containsKey(key)) {
            if (defaultValue instanceof Double) {
                Preferences.initDouble(key, (double) defaultValue);
            } else if (defaultValue instanceof Integer) {
                Preferences.initInt(key, (int) defaultValue);
            } else if (defaultValue instanceof Boolean) {
                Preferences.initBoolean(key, (boolean) defaultValue);
            } else if (defaultValue instanceof String) {
                Preferences.initString(key, (String) defaultValue);
            }
        } else {
            if (defaultValue instanceof Double) {
                holder.entry.setDouble(Preferences.getDouble(key, (double) defaultValue));
            } else if (defaultValue instanceof Integer) {
                holder.entry.setDouble(Preferences.getInt(key, (int) defaultValue));
            } else if (defaultValue instanceof Boolean) {
                holder.entry.setBoolean(Preferences.getBoolean(key, (boolean) defaultValue));
            } else if (defaultValue instanceof String) {
                holder.entry.setString(Preferences.getString(key, (String) defaultValue));
            }
        }

        valueHolders.put(key, holder);
    }

    public static void updateValues() {
        for (ValueHolder<?> holder : valueHolders.values()) {
            if (holder.defaultValue instanceof Double) {
                double newValue = holder.entry.getDouble((double) holder.defaultValue);
                if (newValue != Preferences.getDouble(holder.key, (double) holder.defaultValue)) {
                    Preferences.setDouble(holder.key, newValue);
                    System.out.println("Setting " + holder.key + " to " + newValue);
                    System.out.println("Preferences: " + Preferences.getDouble(holder.key, (double) holder.defaultValue));
                }
            } else if (holder.defaultValue instanceof Integer) {
                int newValue = (int) holder.entry.getInteger((long) holder.defaultValue);
                if (newValue != Preferences.getInt(holder.key, (int) holder.defaultValue)) {
                    Preferences.setInt(holder.key, newValue);
                }
            } else if (holder.defaultValue instanceof Boolean) {
                boolean newValue = holder.entry.getBoolean((boolean) holder.defaultValue);
                if (newValue != Preferences.getBoolean(holder.key, (boolean) holder.defaultValue)) {
                    Preferences.setBoolean(holder.key, newValue);
                }
            } else if (holder.defaultValue instanceof String) {
                String newValue = holder.entry.getString((String) holder.defaultValue);
                if (!newValue.equals(Preferences.getString(holder.key, (String) holder.defaultValue))) {
                    Preferences.setString(holder.key, newValue);
                }
            }
        }
    }

    public static class ValueHolder<T> {
        private String key;
        private T defaultValue;
        private GenericEntry entry;
    }
}
