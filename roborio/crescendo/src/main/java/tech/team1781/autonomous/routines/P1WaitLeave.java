package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class P1WaitLeave implements AutoRoutine{

    @Override
    public String getName() {
        return "8: P1WaitLeave";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                new AutoStep(6, Action.SHOOT_SUBWOOFER),
                new AutoStep(5, Action.WAIT),
                new AutoStep(10, Paths.getPathFromName("p1;leave"))
        };
    }
    
}
