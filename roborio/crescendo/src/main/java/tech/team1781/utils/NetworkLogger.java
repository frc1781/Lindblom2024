package tech.team1781.utils;

import com.fasterxml.jackson.databind.jsontype.impl.TypeDeserializerBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import tech.team1781.ConfigMap;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.HashMap;

public class NetworkLogger {
    private HashMap<String, GenericEntry> tabMap = new HashMap<String, GenericEntry>();

    public void log(String tabName, double data) {
        GenericEntry tab = findTab(tabName, "Double");

        tab.setDouble(new BigDecimal(data).setScale(2, RoundingMode.HALF_UP).doubleValue());
    }

    public void log(String tabName, String data) {
        GenericEntry tab = findTab(tabName, "String");
        tab.setString(data);
    }

    private GenericEntry findTab(String tabName, String type) {
        if (!tabMap.containsKey(tabName)) {
            tabMap.put(tabName, ConfigMap.SHUFFLEBOARD_TAB.add(tabName, type == "Double" ? 0 : "EMPTY" ).getEntry());
        }

        return tabMap.get(tabName);
    }
}
