package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;

public class P3C5C4Score implements AutoRoutine {

    @Override
    public String getName() {
        return "Red 17: P3C5C4Score";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                
                new AutoStep(2.0, Action.COLLECT_RAMP, Paths.getPathFromName("p3;c5")),
                new AutoStep(4,Paths.getPathFromName("c5;shoot")),
                new AutoStep(3, Action.SHOOT_FAR),
                new AutoStep(5,Action.COLLECT_RAMP,Paths.getPathFromName("shoot;c4")),
                new AutoStep(5, Paths.getPathFromName("c4;shoot")),
                new AutoStep(3, Action.SHOOT_FAR),
        };
    }

}
