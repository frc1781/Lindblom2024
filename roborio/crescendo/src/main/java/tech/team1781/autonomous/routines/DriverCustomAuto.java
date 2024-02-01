package tech.team1781.autonomous.routines;

import edu.wpi.first.networktables.GenericEntry;
import tech.team1781.ConfigMap;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousBuilder;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;

public class DriverCustomAuto implements AutoRoutine {
    private GenericEntry mPositionString = ConfigMap.SHUFFLEBOARD_TAB.add("Custom Auto Positions", "").getEntry();

    @Override
    public String getName() {
        return "Driver Custom Auto";
    }

    @Override
    public AutoStep[] getSteps() {
        return AutonomousBuilder.buildFromString(mPositionString.getString(""));
    }
    
}
