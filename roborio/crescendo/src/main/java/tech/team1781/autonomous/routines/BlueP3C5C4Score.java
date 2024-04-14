package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class BlueP3C5C4Score implements AutoRoutine {

    @Override
    public String getName() {
        return "Blue 17: P3C5C4Score";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                new AutoStep(100, Action.OFF_KICKSTAND),
                new AutoStep(0.1, Positions.P3),
                new AutoStep(3, Action.SHOOT_SUBWOOFER_NO_AIM),
                new AutoStep(1.4, EVector.positionWithDegrees(2.2, 0.8, 0)),
                new AutoStep(5, Action.COLLECT_RAMP, Positions.C5, true),
                new AutoStep(1.5, Action.COLLECT_RAMP, EVector.positionWithDegrees(2.2, 2.3, Positions.P3.z)),
                new AutoStep(2.5, Action.COLLECT_RAMP, Positions.P3.withX(Positions.P3.x + 0.25).withY(Positions.P3.y + 0.25)),
                new AutoStep(3, Action.SHOOT_SUBWOOFER_NO_AIM),
                new AutoStep(1.4, EVector.positionWithDegrees(2.2, 0.8, 45)),
                new AutoStep(5, Action.COLLECT_RAMP, Positions.C4.withZ(Math.toRadians(45)).withX(Positions.C4.x + 0.75).withY(Positions.C4.y + 0.25), true),

                // new AutoStep(1.4, EVector.positionWithDegrees(2.2, 0.8, 45)),
                
                        // Positions.P3.withX(Positions.P3.x + 0.5).withY(Positions.P3.y + 0.3)),
        };
    }

}
