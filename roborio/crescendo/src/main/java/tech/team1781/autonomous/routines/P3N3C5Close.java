package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;

public class P3N3C5Close implements AutoRoutine {

    @Override
    public String getName() {
        return "6: P3N3C5Close";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                new AutoStep(6, Action.SHOOT_SUBWOOFER_NO_AIM),
                new AutoStep(5, Action.COLLECT_RAMP, Paths.getPathFromName("p3;n3")),
                new AutoStep(3, Action.SHOOT_NOTE_THREE),
                new AutoStep(5, Action.COLLECT_RAMP, Paths.getPathFromName("n3;c5"), true),
                new AutoStep(5, Action.RAMP_SHOOTER, Paths.getPathFromName("c5;n3")),
                new AutoStep(6, Action.SHOOT_NOTE_THREE)
        };
    }

}
