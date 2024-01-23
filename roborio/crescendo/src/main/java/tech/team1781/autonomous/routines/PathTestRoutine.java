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

public class PathTestRoutine implements AutoRoutine{

    @Override
    public String getName() {
        return "Path Test Routine";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(10,Paths.p2r3Path),
            new AutoStep(10,Paths.r3r2Path),
            new AutoStep(10, Paths.r2r1Path)
        };
    }
    
}
