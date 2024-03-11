package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class P2N2DCSubwoofer implements AutoRoutine{

    @Override
    public String getName() {
        return "P2N2DCSubwoofer";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(2, Action.OFF_KICKSTAND, Positions.P2),
            new AutoStep(5, Action.SHOOT_SUBWOOFER),
            new AutoStep(2, Action.COLLECT_RAMP, EVector.positionWithDegrees(2.89, 5.55, 0)),
            new AutoStep(2, Action.COLLECT_RAMP, Positions.P2),
            new AutoStep(2, Action.SHOOT_SUBWOOFER),
            // new AutoStep(2, EVector.positionWithDegrees(4.32, 5.64, 0)),
            // new AutoStep(2, EVector.positionWithDegrees(5.33, 6.41, 0)),
            // new AutoStep(2, EVector.positionWithDegrees(7.09, 6.29, 0))
        };
    }
    
}
