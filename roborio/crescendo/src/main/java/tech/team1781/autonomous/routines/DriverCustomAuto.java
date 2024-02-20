package tech.team1781.autonomous.routines;

import java.util.LinkedHashMap;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import tech.team1781.ConfigMap;
import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousBuilder;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;

public class DriverCustomAuto implements AutoRoutine {
    private GenericEntry mPositionString = ConfigMap.AUTONOMOUS_TAB.add("Custom Auto Positions", "").getEntry();
    private LinkedHashMap<GenericEntry, Paths.AutonomousPosition> mPositionEntries = new LinkedHashMap<>();
    
    public DriverCustomAuto() {
        for(Paths.AutonomousPosition position : Paths.AutonomousPosition.values()) {
            mPositionEntries.put(ConfigMap.AUTONOMOUS_TAB.add(position.name(), false)
            .withWidget(BuiltInWidgets.kToggleButton)
            .withPosition(position.getX(), position.getY())
            .getEntry()
            , position);
        }
    }

    @Override
    public String getName() {
        return "Driver Custom Auto";
    }

    @Override
    public AutoStep[] getSteps() {
        // return AutonomousBuilder.buildFromString(mPositionString.getString(""));
        return AutonomousBuilder.buildFromLinkedHashMap(mPositionEntries);
    }
    
}
