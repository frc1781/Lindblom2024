package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class Match45Routine implements AutoRoutine{

    @Override
    public String getName() {
        return "Match45Routine";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(100, Action.OFF_KICKSTAND),
            new AutoStep(5, Action.COLLECT_AUTO_SHOOT),
            new AutoStep(5, Action.COLLECT_RAMP, new EVector(1.64, 0, 0)),
            new AutoStep(5, Action.COLLECT_RAMP, new EVector(0, 0, 0)),
            new AutoStep(10, Action.COLLECT_AUTO_SHOOT)
        };
    }
    
}
