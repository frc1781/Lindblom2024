package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class BlueP1C1 implements AutoRoutine {

    @Override
    public String getName() {
        return "Blue 19: P1C1";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                new AutoStep(100, Action.OFF_KICKSTAND),
                new AutoStep(0.1, Positions.P1),
                new AutoStep(2, Action.SHOOT_SUBWOOFER_NO_AIM),
                new AutoStep(2, Action.COLLECT_RAMP_STAY_DOWN),
                new AutoStep(0.75, Action.COLLECT_RAMP_STAY_DOWN, EVector.positionWithDegrees(1.4, 7.7, 0)),
                new AutoStep(1.5, Action.COLLECT_RAMP_STAY_DOWN, EVector.positionWithDegrees(6.6, 7.7, 0)),
                new AutoStep(2.5, Action.COLLECT_RAMP_STAY_DOWN, EVector.positionWithDegrees(6.6, 6.8, 0)),
                new AutoStep(3, Action.COLLECT_RAMP, Positions.C1.withX(Positions.C1.x + 1).withZ(Math.toRadians(225)), true),
                new AutoStep(2.5, Action.COLLECT_RAMP_STAY_DOWN, EVector.positionWithDegrees(3.75, 6.8, 353)),
                new AutoStep(2.5, Action.COLLECT_RAMP, EVector.positionWithDegrees(3.75, 5.15, 353)),
                new AutoStep(2.5, Action.SHOOT_FAR),

        };
    }

}
