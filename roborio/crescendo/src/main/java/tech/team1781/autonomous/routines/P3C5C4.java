package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class P3C5C4 implements AutoRoutine {

    @Override
    public String getName() {
        return "15: P3C5C4";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(100, Action.OFF_KICKSTAND),
            new AutoStep(3, Action.SHOOT_SUBWOOFER_NO_AIM),
            new AutoStep(20, Action.COLLECT_RAMP, Paths.getPathFromName("p3;c5"), true),
                // new AutoStep(1.5, Action.COLLECT_RAMP, Paths.getPathFromName("c5;shoot")),
                // new AutoStep(5, Action.SHOOT_FAR),
                // new AutoStep(5, Action. COLLECT_RAMP, Paths.getPathFromName("shoot;c4"), false),
                // new AutoStep(1.5, Action.COLLECT_RAMP, Paths.getPathFromName("c4;shoot")),
                // new AutoStep(5, Action.SHOOT_FAR),

/*                new AutoStep(1.5, Action.COLLECT_RAMP, EVector.positionWithDegrees(7, 2.3, 0)),
                new AutoStep(5, Action.COLLECT_RAMP, Positions.C4, true),
                new AutoStep(1.5, Action.COLLECT_RAMP, EVector.positionWithDegrees(5.2, 1, 315)),
                new AutoStep(5, Action.SHOOT_FAR),*/
        };
    }

}
