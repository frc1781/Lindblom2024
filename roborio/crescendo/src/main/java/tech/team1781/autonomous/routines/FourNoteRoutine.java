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
                new AutoStep(3, Action.SHOOT),
                new AutoStep(5, Action.COLLECT_AUTO_SHOOT ,Paths.p2n2Path),
                new AutoStep(4, Action.COLLECT_AUTO_SHOOT, Paths.n2n1Path),
                new AutoStep(5, Action.COLLECT_AUTO_SHOOT, Paths.n1n3Path),
                new AutoStep(5, Action.SHOOT)
        };
    }

}
