package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class P3C4C3 implements AutoRoutine {

    @Override
    public String getName() {
        return "16: P3C4C3";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                new AutoStep(100, Action.OFF_KICKSTAND),
                new AutoStep(0.1, Positions.P3),
                new AutoStep(3, Action.SHOOT_SUBWOOFER_NO_AIM),
                new AutoStep(1.4, Action.COLLECT_RAMP_STAY_DOWN,EVector.positionWithDegrees(3.45, 0.8, 45)),
                new AutoStep(1.4, Action.COLLECT_RAMP_STAY_DOWN,EVector.positionWithDegrees(7, 0.8, 45)),
                new AutoStep(1.4, Action.COLLECT_RAMP_STAY_DOWN,EVector.positionWithDegrees(6.77, 2.38, 0)),
                new AutoStep(5, Action.COLLECT_RAMP, Positions.C4, true),
                new AutoStep(1.4, Action.COLLECT_RAMP_STAY_DOWN,EVector.positionWithDegrees(7, 0.8, 315)),
                new AutoStep(1.5, Action.COLLECT_RAMP, EVector.positionWithDegrees(5.2, 1, 315)),
                new AutoStep(5, Action.SHOOT_FAR),
                new AutoStep(1.4, Action.COLLECT_RAMP_STAY_DOWN,EVector.positionWithDegrees(6.6, 1.5, 30)),
                new AutoStep(5, Action.COLLECT_RAMP, Positions.C3, true),
                new AutoStep(1.4, Action.COLLECT_RAMP,EVector.positionWithDegrees(6.6, 1.5, 315)),
                new AutoStep(1.5, Action.COLLECT_RAMP, EVector.positionWithDegrees(5.2, 1, 315)),
                new AutoStep(5, Action.SHOOT_FAR),
        };
    }

}
