package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class P3C5N3 implements AutoRoutine {

    @Override
    public String getName() {
        return "14: P3C5N3";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                
                new AutoStep(3, Action.SHOOT_SUBWOOFER_NO_AIM),
                new AutoStep(2.0, Action.COLLECT_RAMP, Paths.getPathFromName("p3;c5")),
                new AutoStep(4,Paths.getPathFromName("c5;shoot")),
                new AutoStep(3, Action.SHOOT_FAR),
                new AutoStep(4,Action.COLLECT_RAMP,Paths.getPathFromName("shoot;n3")),
                new AutoStep(5, Action.SHOOT_NOTE_THREE)
        };
    }

}
