package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class PIDTuningRoutine implements AutoRoutine {

    @Override
    public String getName() {
        return "PID Tuning Routine";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(5, Action.TEST_ACTION,new EVector(1,0,0)),
            new AutoStep(5, new EVector(0,0,0))
        };
    }
    
}
