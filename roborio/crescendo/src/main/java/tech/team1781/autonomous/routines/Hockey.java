package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class Hockey implements AutoRoutine{

    @Override
    public String getName() {
        return "Hockey";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(100, Action.OFF_KICKSTAND, Positions.P3),
            new AutoStep(100, Action.COLLECT_RAMP),
            new AutoStep(5, Action.SHOOT_SUBWOOFER),
            new AutoStep(10, EVector.positionWithDegrees(3.07, 0.6, 0)),
            new AutoStep(10, Action.COLLECT_RAMP, Positions.C5)
        };
    }
    
}
