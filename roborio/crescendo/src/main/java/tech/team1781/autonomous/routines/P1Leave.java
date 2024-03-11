package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class P1Leave implements AutoRoutine{

    @Override
    public String getName() {
        return "P1Leave";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(100, Action.OFF_KICKSTAND, Positions.P1),
            new AutoStep(5, Action.SHOOT_SUBWOOFER),
            new AutoStep(2, EVector.positionWithDegrees(1.37, 7.55, 0)),
            new AutoStep(5),
            new AutoStep(2, EVector.positionWithDegrees(7.24, 7.53, 0))
        };
    }
    
}
