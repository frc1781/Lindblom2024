package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class ShootAndOut implements AutoRoutine{

    @Override
    public String getName() {
        return "Shoot and Out Routine";
    }

    @Override
    public AutoStep[] getSteps() {
        RedP3Routine.areWeFucked = true;

        return new AutoStep[] {
            new AutoStep(100, Action.OFF_KICKSTAND),
            new AutoStep(5, Action.COLLECT_AUTO_SHOOT),
            new AutoStep(2, Action.COLLECT_RAMP, new EVector(4.2, 0, 0)),
            // .75 4.5 :: 3.2 1.1
        };
    }
    
}
