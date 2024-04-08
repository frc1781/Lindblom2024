package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class TestRoutine implements AutoRoutine {

    @Override
    public String getName() {
        return "Test Routine";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(5, Action.COLLECT_RAMP,EVector.positionWithDegrees(0, 0, 0)),
            new AutoStep(5, Action.COLLECT_RAMP,EVector.positionWithDegrees(2, 0, 0) , true),
        };
    }

}
