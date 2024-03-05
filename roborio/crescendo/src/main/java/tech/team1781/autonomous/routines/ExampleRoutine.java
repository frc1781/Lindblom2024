package tech.team1781.autonomous.routines;

import edu.wpi.first.math.trajectory.Trajectory;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class ExampleRoutine implements AutoRoutine {

    @Override
    public String getName() {
        return "Example Routine";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[]{
            new AutoStep(100, ControlSystem.Action.OFF_KICKSTAND),
        };
    }
    
}
