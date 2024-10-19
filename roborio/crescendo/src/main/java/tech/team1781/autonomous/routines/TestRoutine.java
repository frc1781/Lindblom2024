package tech.team1781.autonomous.routines;

import java.nio.file.Path;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;

public class TestRoutine implements AutoRoutine {

    @Override
    public String getName() {
        return "Test Routine2";
    }

    @Override
    public AutoStep[] getSteps() {  
        return new AutoStep[] {
            new AutoStep(100, Action.SYSID, Paths.getPathFromName("tpath")),
        };
    }

}
