package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class P2N2C2Subwoofer implements AutoRoutine{

    @Override
    public String getName() {
        return "5: P2N2C2Subwoofer";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(100, Action.OFF_KICKSTAND),
            new AutoStep(5, Action.COLLECT_RAMP),
            new AutoStep(0.1, Positions.P2),
            new AutoStep(6, Action.SHOOT_SUBWOOFER),
            new AutoStep(3, Action.COLLECT_RAMP, Positions.N2,true), 
            new AutoStep(2, Action.COLLECT_RAMP, Positions.P2),
            new AutoStep(2, Action.SHOOT_SUBWOOFER),
            new AutoStep(1.5, EVector.positionWithDegrees(4.77, 5.97, 0)),
            new AutoStep(1.5, Action.COLLECT_RAMP, EVector.positionWithDegrees(5.85, 6.51, 5.5)),
            new AutoStep(10, Action.COLLECT_RAMP,Positions.C2.withZ(5.5), true)
        };
    }
    
}
