package tech.team1781.autonomous.routines;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;

public class PathTestRoutine implements AutoRoutine{

    @Override
    public String getName() {
        return "Path Test Routine";
    }

    @Override
    public AutoStep[] getSteps() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("pathplanner");
        PathPlannerTrajectory trajectory = new PathPlannerTrajectory(path, new ChassisSpeeds(0,0,0), new Rotation2d());

        return new AutoStep[] {
            new AutoStep(10, trajectory),
        };
    }
    
}