package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem;

public class SYSIDRoutine implements AutoRoutine{

    @Override
    public String getName() {
        return "System Identification";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(100, ControlSystem.Action.SYSID)
        };
    }
    
}
