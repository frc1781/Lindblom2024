package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class P3Leave implements AutoRoutine{

    @Override
    public String getName() {
        return "7: P3Leave";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(100, Action.OFF_KICKSTAND),
            new AutoStep(100, Action.COLLECT_RAMP),
            new AutoStep(0.1, Positions.P3),
            new AutoStep(5, Action.SHOOT_SUBWOOFER),
            new AutoStep(1.6, EVector.positionWithDegrees(3.2, 0.5, 0)),
            new AutoStep(2, Action.COLLECT_RAMP, Positions.C5, true),
            new AutoStep(1.6, EVector.positionWithDegrees(3.2, 0.5, 0)),
            new AutoStep(2, Positions.P3),
            new AutoStep(6, Action.SHOOT_SUBWOOFER),
        };
    }
    
}
