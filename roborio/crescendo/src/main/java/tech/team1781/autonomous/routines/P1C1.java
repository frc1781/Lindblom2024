package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;

public class P1C1 implements AutoRoutine {

    @Override
    public String getName() {
        return "Red 19: P1C1";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                new AutoStep(100, Action.OFF_KICKSTAND),
                new AutoStep(2, Action.SHOOT_SUBWOOFER_NO_AIM),
                new AutoStep(5, Action.COLLECT_RAMP, Paths.getPathFromName("p1;c1")),
                new AutoStep(5, Action.COLLECT_RAMP, Paths.getPathFromName("c1;shootAmpSide")),
                new AutoStep(2.5, Action.SHOOT_FAR),

        };
    }

}
