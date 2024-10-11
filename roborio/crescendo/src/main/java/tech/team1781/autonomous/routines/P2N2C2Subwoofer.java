package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;

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
            new AutoStep(2, Action.SHOOT_NOTE_TWO),
            new AutoStep(5, Action.COLLECT_RAMP, Paths.getPathFromName("n2;c2")),
            new AutoStep(5, Action.RAMP_SHOOTER, Paths.getPathFromName("c2;n2")),
            new AutoStep(5, Action.SHOOT_NOTE_TWO),
        };
    }
    
}
