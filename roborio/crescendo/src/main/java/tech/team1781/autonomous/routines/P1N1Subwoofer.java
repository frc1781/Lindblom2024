package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class P1N1Subwoofer implements AutoRoutine {

    @Override
    public String getName() {
        return "1: P1N1Subwoofer";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                new AutoStep(100, Action.OFF_KICKSTAND),
                new AutoStep(0.1, Positions.P1),
                new AutoStep(5, Action.SHOOT_SUBWOOFER),
                new AutoStep(2, Action.COLLECT_RAMP, EVector.positionWithDegrees(3.14, 1.71, 0)),
                new AutoStep(2, Action.COLLECT_RAMP,Positions.N1, true), 
                new AutoStep(3, Action.COLLECT_RAMP, Positions.P1),
                new AutoStep(2, Action.AUTO_AIM_SHOOT),
                

        };
    }

}
