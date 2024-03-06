package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.Paths.AutonomousPosition;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;

public class BlueP3Routine implements AutoRoutine{
    public static boolean areWeFucked = false;

    @Override
    public String getName() {
        return "BlueP3Routine";
    }

    @Override
    public AutoStep[] getSteps() {
        areWeFucked = true;
        return new AutoStep[] {
            new AutoStep(100, Action.OFF_KICKSTAND),
            new AutoStep(10, Action.COLLECT_AUTO_SHOOT),
            new AutoStep(10, Action.COLLECT_AUTO_SHOOT ,Paths.getPath(AutonomousPosition.POSITION_3, AutonomousPosition.NOTE_3)),
            new AutoStep(10, Action.AUTO_AIM_SHOOT)
        };
    }
    
}
