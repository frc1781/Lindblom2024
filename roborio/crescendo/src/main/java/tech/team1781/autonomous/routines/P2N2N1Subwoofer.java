package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;

public class P2N2N1Subwoofer implements AutoRoutine{

    @Override
    public String getName() {
        return "10: P2N2N1Subwoofer";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(100, Action.OFF_KICKSTAND),
            new AutoStep(6, Action.SHOOT_SUBWOOFER_NO_AIM),
            new AutoStep(5, Action.COLLECT_RAMP, Paths.getPathFromName("p2;n2")),
            new AutoStep(2, Action.SHOOT_NOTE_TWO),
            new AutoStep(5, Action.COLLECT_RAMP, Paths.getPathFromName("n2;n1")),
            new AutoStep(2, Action.SHOOT_NOTE_ONE),
        };
    }
    
}
