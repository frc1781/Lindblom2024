package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;

public class P1N1 implements AutoRoutine {

    @Override
    public String getName() {
        return "1: P1N1";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(6, Action.SHOOT_SUBWOOFER_NO_AIM),
            new AutoStep(5, Action.COLLECT_RAMP, Paths.getPathFromName("p1;n1")),
            new AutoStep(5, Action.SHOOT_NOTE_ONE),
        };
    }

}
