package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.autonomous.Positions;
import tech.team1781.control.ControlSystem;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

import java.nio.file.Path;

public class TestRoutine implements AutoRoutine {

    @Override
    public String getName() {
        return "Test Routine";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                new AutoStep(100, Action.OFF_KICKSTAND),
                new AutoStep(6, Action.SHOOT_SUBWOOFER),
                new AutoStep(2,Action.COLLECT_RAMP),

/*
                new AutoStep(0.1, Positions.P2),

                new AutoStep(3, Action.COLLECT_RAMP, Positions.N2, true),*/

                new AutoStep(1.5, Action.COLLECT_RAMP, Paths.getPathFromName("p2;n2"), true),
                new AutoStep(1.5, Action.COLLECT_RAMP, Paths.getPathFromName("n2;p2")),
                new AutoStep(2, Action.SHOOT_SUBWOOFER),
                new AutoStep(1.5, Action.COLLECT_RAMP, Paths.getPathFromName("p2;n3"), true),
                new AutoStep(1.5, Action.COLLECT_RAMP, Paths.getPathFromName("n3;p2")),
                new AutoStep(2, Action.SHOOT_SUBWOOFER),
                new AutoStep(1.5, Action.COLLECT_RAMP, Paths.getPathFromName("p2;n1"), true),
                new AutoStep(2, Action.COLLECT_RAMP, Paths.getPathFromName("n1;p2")),
                new AutoStep(2, Action.SHOOT_SUBWOOFER)


/*                new AutoStep(2, Action.SHOOT_NOTE_TWO),
                new AutoStep(2, Math.toRadians(90)),
                new AutoStep(2, Action.COLLECT_RAMP, Positions.N1.withZ(Math.toRadians(90)), true),
                new AutoStep(2, Action.COLLECT_RAMP ,Positions.P2),
                new AutoStep(6, Action.SHOOT_SUBWOOFER),
                // new AutoStep(1.5, Action.COLLECT_RAMP, EVector.positionWithDegrees(2.0, 4.1, 0)),
                // new AutoStep(1.5, Action.COLLECT_RAMP, EVector.positionWithDegrees(1.8, 3.9, 0)),
                new AutoStep(3, Action.COLLECT_RAMP_STAY_DOWN, Positions.N3.withZ(Math.toRadians(315)), true),
                new AutoStep(1.5, Action.COLLECT_RAMP, EVector.positionWithDegrees(2.0, 4.1, 308)),
                // new AutoStep(2, Action.COLLECT_RAMP, Positions.P2),
                new AutoStep(5, Action.SHOOT_NOTE_THREE),*/
        };
    }

}
