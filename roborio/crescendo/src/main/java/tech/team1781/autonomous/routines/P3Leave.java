package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class P3Leave implements AutoRoutine{

    @Override
    public String getName() {
        return "P3Leave";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(100, Action.OFF_KICKSTAND, Positions.P3),
            new AutoStep(5, Action.SHOOT_SUBWOOFER),
            // new AutoStep(2, EVector.newVector(2.82, 1.85, 0))
        };
    }
    
}
