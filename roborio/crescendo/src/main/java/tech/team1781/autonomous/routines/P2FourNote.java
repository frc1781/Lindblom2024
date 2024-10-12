package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem;
import tech.team1781.control.ControlSystem.Action;

public class P2FourNote implements AutoRoutine {

    @Override
    public String getName() {
        return "13: P2FourNote";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                new AutoStep(6, Action.SHOOT_SUBWOOFER_NO_AIM),
                new AutoStep(5, Action.COLLECT_RAMP, Paths.getPathFromName("p2;n2"), true, true),
                new AutoStep(5, Action.SHOOT_NOTE_TWO),
                new AutoStep(5, Action.COLLECT_RAMP_STAY_DOWN, Paths.getPathFromName("n2;n3"), true, true),
                new AutoStep(5, Action.COLLECT_RAMP_STAY_DOWN, Paths.getPathFromName("n3;p2")),
                new AutoStep(5, Action.SHOOT_SUBWOOFER_NO_AIM),
                new AutoStep(5, Action.COLLECT_RAMP, Paths.getPathFromName("p2;n1;simple"), true, true),
                new AutoStep(5, Action.RAMP_SHOOTER, Paths.getPathFromName("n1;p2;simple")),
                new AutoStep(5, Action.SHOOT_SUBWOOFER_NO_AIM)
        };
    }
} 