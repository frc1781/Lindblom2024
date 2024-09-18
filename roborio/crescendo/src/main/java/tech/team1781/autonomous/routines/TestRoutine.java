package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;

public class TestRoutine implements AutoRoutine {

    @Override
    public String getName() {
        return "Test Routine2";
    }

    @Override
    public AutoStep[] getSteps() {  
        return new AutoStep[] {
            new AutoStep(10, Paths.getPathFromName("tpath"))
        };
    }

}
