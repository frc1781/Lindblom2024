package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class P1C1C2 implements AutoRoutine {

    @Override
    public String getName() {
        return "16: P1C1C2";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                new AutoStep(100, Action.OFF_KICKSTAND),
                new AutoStep(0.1, Positions.P1),
                new AutoStep(5, Action.SHOOT_SUBWOOFER_NO_AIM),
                new AutoStep(1.4, Action.COLLECT_RAMP_STAY_DOWN,EVector.positionWithDegrees(1.4, 7.7, 0)),
                new AutoStep(1.4, Action.COLLECT_RAMP_STAY_DOWN, EVector.positionWithDegrees(6.6, 7.7, 0)),
                new AutoStep(1.4, Action.COLLECT_RAMP_STAY_DOWN,EVector.positionWithDegrees(16, 4.5, 0)),
                new AutoStep(1, Action.COLLECT_RAMP, Positions.C1, true),

        };
    }

}
