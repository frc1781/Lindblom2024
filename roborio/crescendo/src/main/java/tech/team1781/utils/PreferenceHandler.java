package tech.team1781.utils;

import java.util.HashMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Preferences;
import tech.team1781.ConfigMap;

public class PreferenceHandler {
    private HashMap<String, ValueHolder<?>> mValues = new HashMap<>();

    public PreferenceHandler(ValueHolder<?>... values) {
        for (ValueHolder<?> value : values) {
            mValues.put(value.key, value);
            value.shuffleboardEntry = ConfigMap.SHUFFLEBOARD_TAB.add(value.key, value.value).getEntry();

            if (value.value instanceof Double) {
                Preferences.initDouble(value.key, (double) value.value);
            }        
        }
    }

    public void checkForUpdates() {
        for (ValueHolder<?> value : mValues.values()) {
            if(value.value instanceof Double){
                double shuffleboardValue = value.shuffleboardEntry.getDouble((double) value.value);
                if (shuffleboardValue != (double) value.value) {
                    Preferences.setDouble(value.key, shuffleboardValue);
                }
            }
        }
    } 

    



    
    public static class ValueHolder<ValueType> {
        public String key;
        public ValueType value;
        public GenericEntry shuffleboardEntry;

        public ValueHolder(String key, ValueType value) {
            this.key = key;
            this.value = value;
        }
    }
}
