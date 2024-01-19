package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class TestPathRoutine implements AutoRoutine{

    @Override
    public String getName() {
        return "Test Path Routine";
    }

    @Override
    public AutoStep[] getSteps() {
        PathPlannerPath path = PathPlannerPath.fromPathFile("OneMeter");
        PathPlannerTrajectory trajectory = new PathPlannerTrajectory(path, new ChassisSpeeds(0,0,0));

        return new AutoStep[] {
            new AutoStep(10, trajectory)
        };
    }
    
}
