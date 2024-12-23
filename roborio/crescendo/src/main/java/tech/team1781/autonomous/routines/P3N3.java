package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;

public class P3N3 implements AutoRoutine{

    @Override
    public String getName() {
        return "3: P3N3";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(6, Action.SHOOT_SUBWOOFER_NO_AIM),
            new AutoStep(5, Action.COLLECT_RAMP, Paths.getPathFromName("p3;n3")),
            new AutoStep(5, Paths.getPathFromName("n3;p3")),
            new AutoStep(5, Action.SHOOT_SUBWOOFER_NO_AIM)
        };  
    }
    
}
