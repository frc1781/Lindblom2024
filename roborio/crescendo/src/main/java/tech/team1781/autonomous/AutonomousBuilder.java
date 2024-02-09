package tech.team1781.autonomous;

import java.util.LinkedList;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import tech.team1781.Paths;

public class AutonomousBuilder {

    public static AutoStep[] buildFromString(String s) {
        s = s.trim();
        String[] steps = s.split(",");
        if(steps.length == 0) {
            throw new IllegalArgumentException("Invalid string format");
        }

        LinkedList<AutoStep> autonomousSteps = new LinkedList<>();
        String previous = steps[0];

        for(int i = 1; i < steps.length; i++) {
            String current = steps[i];
            if(isWait(current)) {
                String timeInt = current.substring(1);
                int time = Integer.parseInt(timeInt);
                AutoStep waitStep = new AutoStep(time);
                autonomousSteps.add(waitStep);
                continue;

            }

           AutoStep currentStep;
           Paths.AutonomousPosition prevPosition = Paths.getPosition(previous);
           Paths.AutonomousPosition nextPosition = Paths.getPosition(current);

           PathPlannerPath path = Paths.getPath(prevPosition, nextPosition);
           System.out.println("Path: " + path.toString() + " ================================================================== ");

           PathPlannerTrajectory tempTraj = path.getTrajectory(new ChassisSpeeds(), new Rotation2d());
           currentStep = new AutoStep(tempTraj.getTotalTimeSeconds() + 0.5, path);
           autonomousSteps.add(currentStep);
           previous = steps[i];
           
        }      
        
        AutoStep[] ret_val = new AutoStep[autonomousSteps.size()];

        for(int i = 0 ; i < autonomousSteps.size(); i ++) {
            ret_val[i] = autonomousSteps.get(i);
        }

        return ret_val;
    }

    private static boolean isWait(String s) {
        return s.startsWith("w");
    }
}
