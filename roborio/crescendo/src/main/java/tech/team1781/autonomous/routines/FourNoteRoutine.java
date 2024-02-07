package tech.team1781.autonomous.routines;

import java.util.ResourceBundle.Control;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem;
import tech.team1781.control.ControlSystem.Action;

public class FourNoteRoutine implements AutoRoutine {

    @Override
    public String getName() {
        return "4 Note Routine";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                // new AutoStep(2, ControlSystem.Action.AUTO_AIM_SHOOT),
                // new AutoStep(2, ControlSystem.Action.COLLECT, Paths.p2r3Path),
                // new AutoStep(2, ControlSystem.Action.AUTO_AIM_SHOOT),
                // new AutoStep(2, ControlSystem.Action.COLLECT, Paths.r3r2Path),
                // new AutoStep(2, ControlSystem.Action.AUTO_AIM_SHOOT),
                // new AutoStep(2, ControlSystem.Action.COLLECT, Paths.r2r1Path),
                // new AutoStep(2, ControlSystem.Action.AUTO_AIM_SHOOT),
        };
    }

}
