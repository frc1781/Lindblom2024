package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;
import tech.team1781.Paths;

public class RedP1SeekRoutine implements AutoRoutine{

    @Override
    public String getName() {
        return "RedP1Routine";
    }

    @Override
    public AutoStep[] getSteps() {
        RedP3Routine.areWeFucked = true;

        return new AutoStep[] {
            new AutoStep(100, Action.OFF_KICKSTAND),
            new AutoStep(5, Action.COLLECT_AUTO_SHOOT),
            new AutoStep(5, Action.COLLECT_RAMP, new EVector(1.32, 0, 0)), // 
            new AutoStep(10, Action.SEEK_NOTE),
            new AutoStep(10, Action.SHOOT_NOTE_ONE)
        };
    }
    
}
