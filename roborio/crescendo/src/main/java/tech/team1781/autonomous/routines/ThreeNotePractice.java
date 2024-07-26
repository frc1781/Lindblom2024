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

public class ThreeNotePractice implements AutoRoutine {

    @Override
    public String getName() {
        return "21: ThreeNotePractice";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
       new AutoStep(3, Paths.getPathFromName("threeNote1")),
       new AutoStep(3, Paths.getPathFromName("threeNote2")),
       new AutoStep(3, Paths.getPathFromName("threeNote3")),
       new AutoStep(3, Paths.getPathFromName("threeNote4")),
       new AutoStep(3, Paths.getPathFromName("threeNote5")),
       new AutoStep(3, Paths.getPathFromName("threeNote6"))
        };
    }
}
