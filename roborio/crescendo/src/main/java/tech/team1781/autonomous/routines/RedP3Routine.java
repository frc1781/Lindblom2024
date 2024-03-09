package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.Paths.AutonomousPosition;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class RedP3Routine implements AutoRoutine{
    public static boolean areWeFucked = false;

    @Override
    public String getName() {
        return "RedP3Routine";
    }

    @Override
    public AutoStep[] getSteps() {
        areWeFucked = true;
        return new AutoStep[] {
            new AutoStep(100, Action.OFF_KICKSTAND),
            new AutoStep(5, Action.COLLECT_AUTO_SHOOT),
            new AutoStep(5, Action.COLLECT_RAMP ,new EVector(1.32, -1.63, 5.3)), //52 64
            new AutoStep(2, new EVector(0, 0, 0)),
            new AutoStep(10, Action.COLLECT_AUTO_SHOOT)  
        };
    }
    
}
