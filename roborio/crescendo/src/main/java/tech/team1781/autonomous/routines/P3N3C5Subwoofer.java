package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class P3N3C5Subwoofer implements AutoRoutine {

    @Override
    public String getName() {
        return "6: P3N3C5Subwoofer";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                
                // new AutoStep(6, Action.SHOOT_SUBWOOFER_NO_AIM),
                // new Autostep( 6, Action       )
                // new AutoStep(4, Action.AUTO_AIM_SHOOT),
                // new AutoStep(1.6, EVector.positionWithDegrees(5.2, 0.5, 0)),
                // new AutoStep(2, Action.COLLECT_RAMP, EVector.positionWithDegrees(8, 0.72, 0)),
                // new AutoStep(1, Action.COLLECT_RAMP, Positions.C5, true),
                // new AutoStep(1.5, EVector.positionWithDegrees(3.2, 0.5, 0)),
                // new AutoStep(2, Positions.P3),
                // new AutoStep(6, Action.SHOOT_SUBWOOFER),
        };
    }

}
