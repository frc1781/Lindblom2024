package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.Paths.AutonomousPosition;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class RedP3SeekRoutine implements AutoRoutine{

    @Override
    public String getName() {
        return "Red P3 Seek";
    }

    @Override
    public AutoStep[] getSteps() {
        RedP3Routine.areWeFucked = true;

        return new AutoStep[] {
            new AutoStep(100, Action.OFF_KICKSTAND),
            new AutoStep(5, Action.COLLECT_AUTO_SHOOT),
            new AutoStep(5, Action.COLLECT_RAMP ,new EVector(1.32, 0, 0)), //52 64 5.3
            new AutoStep(10, Action.SEEK_NOTE),
            new AutoStep(10, Action.AUTO_AIM_SHOOT)
        };
    }
    
}
