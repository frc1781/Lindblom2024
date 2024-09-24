package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class P2N2C2Subwoofer implements AutoRoutine{

    @Override
    public String getName() {
        return "5: P2N2C2Subwoofer";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(6, Action.SHOOT_SUBWOOFER),
            new AutoStep(5, Action.COLLECT_RAMP, Paths.getPathFromName("p2;n2")),
            new AutoStep(2, Action.SHOOT_SUBWOOFER),
            new AutoStep(10, Action.COLLECT_RAMP, Paths.getPathFromName("n2;c2"))
        };
    }
    
}
