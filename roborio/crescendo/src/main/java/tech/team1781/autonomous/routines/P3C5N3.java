package tech.team1781.autonomous.routines;

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
                new AutoStep(100, Action.OFF_KICKSTAND),
                new AutoStep(0.1, Positions.P3),
                new AutoStep(3, Action.SHOOT_SUBWOOFER),
                new AutoStep(1.4, EVector.positionWithDegrees(3.14, 1.71, 0)),
                new AutoStep(5, Action.COLLECT_RAMP, Positions.C5, true),
                new AutoStep(1.5, Action.COLLECT_RAMP, EVector.positionWithDegrees(3.0, 1.71, 0)),
                // new AutoStep(5, Action.COLLECT_RAMP, Positions.P3),
                // option for not quite P3
                new AutoStep(2.5, Action.COLLECT_RAMP, Positions.P3),
                new AutoStep(3, Action.SHOOT_SUBWOOFER),
                new AutoStep(1.3, Action.COLLECT_RAMP, EVector.positionWithDegrees(2.0, 4.1, 0)),
                new AutoStep(3, Action.COLLECT_RAMP_STAY_DOWN, Positions.N3.withX(Positions.N3.x + 0.25), true),
                new AutoStep(1.3, Action.COLLECT_RAMP, Positions.P3),
                // new AutoStep(2, Action.COLLECT_RAMP, Positions.P2),
                new AutoStep(5, Action.SHOOT_SUBWOOFER)
        };
    }

}
