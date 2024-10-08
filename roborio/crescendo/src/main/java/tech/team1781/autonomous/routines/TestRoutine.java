package tech.team1781.autonomous.routines;

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
            new AutoStep(10, Action.COLLECT_RAMP, Paths.getPathFromName("tpath"), true),
            new AutoStep(10, Action.RAMP_SHOOTER, Paths.getPathFromName("tpathBack")),
            new AutoStep(4, Action.SHOOT_SUBWOOFER_NO_AIM)
        };
    }

}
