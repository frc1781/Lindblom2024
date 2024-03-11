package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem;
import tech.team1781.utils.EVector;

public class TestRoutine implements AutoRoutine {

    @Override
    public String getName() {
        return "Test Routine";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                new AutoStep(3, ControlSystem.Action.OFF_KICKSTAND,new EVector(5, 2, Math.PI)),
                new AutoStep(3, new EVector(7, 0, Math.PI)),
        };
    }

}
