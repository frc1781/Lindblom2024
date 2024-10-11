package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;

public class TestP2N2 implements AutoRoutine {
    @Override
    public String getName() {
        return "Test P2N2N1N3";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(6, Action.SHOOT_SUBWOOFER_NO_AIM),
            new AutoStep(10, Action.COLLECT_RAMP, Paths.getPathFromName("p2;n2")),
            new AutoStep(4, Action.SHOOT_NOTE_TWO),
            new AutoStep(10, Action.COLLECT_RAMP, Paths.getPathFromName("n2;n1")),
            new AutoStep(10, Action.SHOOT_NOTE_ONE),
            new AutoStep(10, Action.COLLECT_RAMP, Paths.getPathFromName("n1;n3")),
            new AutoStep(10, Action.SHOOT_NOTE_THREE)
        };
    }
}
