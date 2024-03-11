package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class P1N1CSubwoofer implements AutoRoutine {

    @Override
    public String getName() {
        return "P1N1CSubwoofer";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                new AutoStep(5, Action.OFF_KICKSTAND, EVector.positionWithDegrees(0.94, 6.58, 48.18)),
                new AutoStep(5, Action.SHOOT_SUBWOOFER),
                new AutoStep(5, Action.COLLECT_RAMP, EVector.positionWithDegrees(2.89, 6.58, 0)),
                new AutoStep(5, Action.COLLECT_RAMP, EVector.positionWithDegrees(0.94, 6.58, 48.18)),
                new AutoStep(5, Action.SHOOT_SUBWOOFER),
                new AutoStep(5, EVector.positionWithDegrees(7.57, 7.42, 0))
        };
    }

}
