package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;

public class P1N1C1Far implements AutoRoutine {

    @Override
    public String getName() {
        return "13: P1N1C1Far";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                new AutoStep(5, Action.SHOOT_SUBWOOFER_NO_AIM),
                new AutoStep(5, Action.COLLECT_RAMP, Paths.getPathFromName("p1;n1")),
                new AutoStep(6, Action.SHOOT_NOTE_ONE),
                new AutoStep(5, Action.COLLECT_RAMP, Paths.getPathFromName("n1;c1")),
                new AutoStep(5, Action.RAMP_SHOOTER, Paths.getPathFromName("c1;shootAmpSide")),
                new AutoStep(4, Action.SHOOT_FAR),
        };
    }

}