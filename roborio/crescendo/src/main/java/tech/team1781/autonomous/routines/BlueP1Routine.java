package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.Paths;

public class BlueP1Routine implements AutoRoutine{

    @Override
    public String getName() {
        return "BlueP1Routine";
    }

    @Override
    public AutoStep[] getSteps() {
        BlueP3Routine.areWeFucked = true;
        return new AutoStep[] {
            new AutoStep(100, Action.OFF_KICKSTAND),
            new AutoStep(10, Action.COLLECT_RAMP),
            new AutoStep(10, Action.COLLECT_RAMP, Paths.getPath(Paths.AutonomousPosition.POSITION_1, Paths.AutonomousPosition.NOTE_1)),
            new AutoStep(10, Action.AUTO_AIM_SHOOT)
        };
    }
    
}
