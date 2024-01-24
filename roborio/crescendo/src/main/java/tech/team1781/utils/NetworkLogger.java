package tech.team1781.utils;

import edu.wpi.first.networktables.GenericEntry;
import tech.team1781.ConfigMap;
import java.util.HashMap;

public class NetworkLogger {
    private HashMap<String, GenericEntry> tabMap = new HashMap<String, GenericEntry>();

    public void log(String tabName, double data) {
        GenericEntry tab = findTab(tabName);
        tab.setString(String.valueOf(data));
    }

    public void log(String tabName, String data) {
        GenericEntry tab = findTab(tabName);
        tab.setString(data);
    }

    private GenericEntry findTab(String tabName) {
        if (!tabMap.containsKey(tabName)) {
            System.out.println(tabName);
            tabMap.put(tabName, ConfigMap.SHUFFLEBOARD_TAB.add(tabName, "EMPTY").getEntry());
        }

        return tabMap.get(tabName);
    }
}
