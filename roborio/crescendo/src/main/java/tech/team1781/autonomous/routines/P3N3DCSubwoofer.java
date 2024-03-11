package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class P3N3DCSubwoofer implements AutoRoutine{

    @Override
    public String getName() {
        return "P3N3DCSubwoofer";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(2, Action.OFF_KICKSTAND, Positions.P3),
            new AutoStep(5, Action.SHOOT_SUBWOOFER),
            new AutoStep(2, Action.COLLECT_RAMP, EVector.positionWithDegrees(2.62, 4.11, 0)),
            new AutoStep(3, Action.COLLECT_RAMP, Positions.P3),
            new AutoStep(2, Action.SHOOT_SUBWOOFER),
            // new AutoStep(2, EVector.positionWithDegrees(2.82, 1.85, 0)),j
            // new AutoStep(2, EVector.positionWithDegrees(7.56, 0.8, 0))
        };
    }
    
}
