package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;

public class P3C4C3 implements AutoRoutine {

    @Override
    public String getName() {
        return "16: P3C4C3";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                new AutoStep(100, Action.OFF_KICKSTAND),
                new AutoStep(3, Action.SHOOT_SUBWOOFER_NO_AIM),
                new AutoStep(5, Action. COLLECT_RAMP, Paths.getPathFromName("p3;c4"), true),
                new AutoStep(1.5, Action.RAMP_SHOOTER, Paths.getPathFromName("c4;shoot")),
                new AutoStep(5, Action.SHOOT_FAR),
                new AutoStep(4, Action.COLLECT_RAMP, Paths.getPathFromName("shoot;c3"), true),
                new AutoStep(4, Action.RAMP_SHOOTER, Paths.getPathFromName("c3;shoot")),
                new AutoStep(5, Action.SHOOT_FAR)
        };
    }

}
