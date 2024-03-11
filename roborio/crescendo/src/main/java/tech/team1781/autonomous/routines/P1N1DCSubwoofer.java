package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class P1N1DCSubwoofer implements AutoRoutine {

    @Override
    public String getName() {
        return "P1N1DCSubwoofer";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                new AutoStep(2, Action.OFF_KICKSTAND, Positions.P1),
                new AutoStep(5, Action.SHOOT_SUBWOOFER),
                new AutoStep(2, Action.COLLECT_RAMP,Positions.N1), 
                new AutoStep(3, Action.COLLECT_RAMP, Positions.P1),
                new AutoStep(2, Action.SHOOT_SUBWOOFER),
                // new AutoStep(2, EVector.positionWithDegrees(7.57, 7.42, 0)),
                

        };
    }

}
