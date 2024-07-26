package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.autonomous.Positions;
import tech.team1781.control.ControlSystem;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.subsystems.Arm.ArmState;
import tech.team1781.utils.EVector;

import java.nio.file.Path;

import com.pathplanner.lib.path.PathPlannerPath;

public class BluePractice implements AutoRoutine {

    @Override
    public String getName() {
        return "20: BluePractice";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
       new AutoStep(5, Paths.getPathFromName("bluePractice1")),
       new AutoStep(2,Action.COLLECT_RAMP),
       new AutoStep(5, Paths.getPathFromName("bluePractice2")),
       new AutoStep(5, Paths.getPathFromName("bluePractice4")),
       new AutoStep(5, Paths.getPathFromName("bluePractice3"))
        };
    }
}
