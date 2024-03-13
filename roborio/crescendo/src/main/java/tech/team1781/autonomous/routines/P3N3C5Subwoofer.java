package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class P3N3C5Subwoofer implements AutoRoutine{

    @Override
    public String getName() {
        return "6: P3N3C5Subwoofer";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(100, Action.OFF_KICKSTAND),
            new AutoStep(100, Action.COLLECT_RAMP),
            new AutoStep(6, Action.SHOOT_SUBWOOFER),
            new AutoStep(2, Action.COLLECT_RAMP, Positions.N3, true), 
            new AutoStep(3, Action.COLLECT_RAMP, Positions.P3),
            new AutoStep(4, Action.AUTO_AIM_SHOOT),
            new AutoStep(5, EVector.positionWithDegrees(3.2, 1.72, 0)),
            new AutoStep(10, Action.COLLECT_RAMP, Positions.C5, true)
        };
    }
    
}
