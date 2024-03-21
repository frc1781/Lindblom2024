package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class P2N2N3Subwoofer implements AutoRoutine{

    @Override
    public String getName() {
        return "9: P2N2N3Subwoofer";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(100, Action.OFF_KICKSTAND),
            new AutoStep(0.1, Positions.P2),
            new AutoStep(6, Action.SHOOT_SUBWOOFER),
            new AutoStep(3, Action.COLLECT_RAMP, Positions.N2,true), 
            new AutoStep(2, Action.COLLECT_RAMP, Positions.P2),
            new AutoStep(2, Action.SHOOT_SUBWOOFER),
            new AutoStep(1.5, Action.COLLECT_RAMP, EVector.positionWithDegrees(2.0, 4.1, 0)),
            new AutoStep(3, Action.COLLECT_RAMP_STAY_DOWN, Positions.N3,true), 
            new AutoStep(1.3, Action.COLLECT_RAMP, Positions.P2),
            // new AutoStep(2, Action.COLLECT_RAMP, Positions.P2),
            new AutoStep(5, Action.SHOOT_SUBWOOFER),
        };
    }
    
}
