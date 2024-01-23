package tech.team1781.utils;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import org.opencv.dnn.Net;
import tech.team1781.ConfigMap;
import tech.team1781.subsystems.Subsystem;

import java.io.ObjectInputFilter;
import java.util.HashMap;
import java.util.Map;

public class NetworkLogger {
    private HashMap<String, GenericEntry> tabMap = new HashMap<String, GenericEntry>();

    public void log(String tabName, double data) {
        GenericEntry tab = findTab(tabName);
        tab.setDouble(data);
    }

    public void log(String tabName, String data) {
        GenericEntry tab = findTab(tabName);
        tab.setString(data);
    }

    private GenericEntry findTab(String tabName) {
        if (!tabMap.containsKey(tabName)) {
            tabMap.put(tabName, ConfigMap.SHUFFLEBOARD_TAB.add(tabName, 0).getEntry());
        }

        return tabMap.get(tabName);
    }
}
