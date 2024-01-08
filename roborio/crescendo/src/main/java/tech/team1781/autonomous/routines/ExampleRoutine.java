package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;

public class ExampleRoutine implements AutoRoutine {

    @Override
    public String getName() {
        return "Example Routine";
    }

    @Override
    public AutoStep[] getSteps() {
        throw new UnsupportedOperationException("Unimplemented method 'getSteps'");
    }
    
}
