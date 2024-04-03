package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;
import tech.team1781.autonomous.Positions;

public class TestRoutine implements AutoRoutine {

    @Override
    public String getName() {
        return "Test Routine";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            //      new AutoStep(100,Action.OFF_KICKSTAND),
          new AutoStep(0.1, Positions.P2),
          new AutoStep(2, Positions.N2),
        };
    }

}
