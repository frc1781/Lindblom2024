package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class P3C4C3Score implements AutoRoutine {

    @Override
    public String getName() {
        return "Red 18: P3C4C3Score";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                new AutoStep(100, Action.OFF_KICKSTAND),
                new AutoStep(0.1, Positions.P3),
                new AutoStep(3, Action.SHOOT_SUBWOOFER_NO_AIM),
                new AutoStep(1.4,Action.COLLECT_RAMP_STAY_DOWN ,EVector.positionWithDegrees(2.2, 0.8, 45)),
                new AutoStep(1.4, Action.COLLECT_RAMP_STAY_DOWN,EVector.positionWithDegrees(6.3, 1.0, 45)),
                new AutoStep(5, Action.COLLECT_RAMP, Positions.C4.withZ(Math.toRadians(45)), true),
                new AutoStep(1.4,Action.COLLECT_RAMP,EVector.positionWithDegrees(2.2, 0.8, 0)),
                new AutoStep(2.5, Action.COLLECT_RAMP, Positions.P3),
                new AutoStep(3, Action.SHOOT_SUBWOOFER_NO_AIM),
                new AutoStep(1.4, EVector.positionWithDegrees(2.2, 0.8, 45)),
                new AutoStep(1.4, Action.COLLECT_RAMP_STAY_DOWN,EVector.positionWithDegrees(6.3, 1.0, 45)),
                new AutoStep(1.4, Action.COLLECT_RAMP_STAY_DOWN,EVector.positionWithDegrees(7.5, 4.0, 45)),
                new AutoStep(5, Action.COLLECT_RAMP, Positions.C3.withZ(Math.toRadians(45)), true),

                // new AutoStep(1.4, EVector.positionWithDegrees(2.2, 0.8, 45)),
                
                        // Positions.P3.withX(Positions.P3.x + 0.5).withY(Positions.P3.y + 0.3)),
        };
    }

}
