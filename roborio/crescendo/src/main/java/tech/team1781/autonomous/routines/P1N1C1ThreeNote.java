package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;

class P1N1C1ThreeNote implements AutoRoutine {

    @Override
    public String getName() {
        return "13: P1N1C1ThreeNote";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                new AutoStep(100, Action.OFF_KICKSTAND),
                new AutoStep(5, Action.COLLECT_RAMP),
                new AutoStep(0.1, Positions.P1),
                new AutoStep(5, Action.SHOOT_SUBWOOFER),
                new AutoStep(2, Action.COLLECT_RAMP, Positions.N1, true),
                new AutoStep(2, Action.SHOOT_NOTE_ONE),
                new AutoStep(4, Action.COLLECT_RAMP, Positions.C1, true),
                new AutoStep(3, Action.COLLECT_RAMP, Positions.N1.withZ((41/180) * Math.PI)),
                new AutoStep(2, Action.SHOOT_NOTE_ONE),
        };
    }

}