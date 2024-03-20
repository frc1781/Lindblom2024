package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class P5C5N3 implements AutoRoutine{

    @Override
    public String getName() {
        return "14: P5C5N3";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(100, Action.OFF_KICKSTAND),
            new AutoStep(0.1, Positions.P5),
            new AutoStep(3, Action.SHOOT_SUBWOOFER),
            new AutoStep(5, Action.COLLECT_RAMP, Positions.C5, true),
            new AutoStep(5, EVector.positionWithDegrees(0.8, 4.5, 125)),
            new AutoStep(3, Action.SHOOT_SUBWOOFER)
        };
    }

    
}
