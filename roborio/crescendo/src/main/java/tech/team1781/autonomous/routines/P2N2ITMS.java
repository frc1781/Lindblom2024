package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.autonomous.Positions;
import tech.team1781.control.ControlSystem.Action;

public class P2N2ITMS implements AutoRoutine {

    @Override
    public String getName() {
        return "ITSM P2 N2";
    }

    @Override
    public AutoStep[] getSteps() {
       return new AutoStep[] {
            new AutoStep(6, Action.SHOOT_SUBWOOFER),
            new AutoStep(5, Action.COLLECT, Paths.getPathFromName("p2;n2")),
       };
    }
    
}
