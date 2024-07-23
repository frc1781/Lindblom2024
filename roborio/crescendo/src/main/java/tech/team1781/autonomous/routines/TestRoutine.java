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
             new AutoStep(10,Paths.getPathFromName("p4;n4")),
             new AutoStep(10,Paths.getPathFromName("n4;p4")),
             //new AutoStep(10,Paths.getPathFromName("p4;n5")),
             //new AutoStep(10,Paths.getPathFromName("n5;p4")),
        };
    }

}
