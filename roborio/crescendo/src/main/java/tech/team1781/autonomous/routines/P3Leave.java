package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;

public class P3Leave implements AutoRoutine{

    @Override
    public String getName() {
        return "7: P3Leave";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(100, Action.OFF_KICKSTAND),
            new AutoStep(5, Action.SHOOT_SUBWOOFER_NO_AIM),
            new AutoStep(4, Action.COLLECT_RAMP,Paths.getPathFromName("p3;leave")),
        };
    }
    
}
