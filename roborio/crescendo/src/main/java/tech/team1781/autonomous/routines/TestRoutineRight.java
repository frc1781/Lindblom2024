package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.autonomous.Positions;
import tech.team1781.control.ControlSystem;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

import java.nio.file.Path;

public class TestRoutineRight implements AutoRoutine {

    @Override
    public String getName() {
        return "Test Routine Right";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                new AutoStep(5, Paths.getPathFromName("pathRight")),

        };
    }

}
