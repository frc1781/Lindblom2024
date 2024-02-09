package tech.team1781.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;

import tech.team1781.Paths;

public class AutonomousBuilder {

    public static AutoStep[] buildFromString(String s) {
        s = "p1,n2,n3,n1";
        s = s.trim();
        String[] steps = s.split(",");
        if(steps.length == 0 || steps.length == 1) {
            throw new IllegalArgumentException("Invalid string format");
        }

        AutoStep[] autonomousSteps = new AutoStep[steps.length];
        String previous = steps[0];

        for(int i = 1; i < steps.length; i++) {
            String current = steps[i];
            if(isWait(current)) {
                String timeInt = current.substring(1);
                int time = Integer.parseInt(timeInt);
                AutoStep waitStep = new AutoStep(time);
                autonomousSteps[i] = waitStep;
                continue;

            }

           AutoStep currentStep;
           Paths.AutonomousPosition prevPosition = Paths.getPosition(previous);
           Paths.AutonomousPosition nextPosition = Paths.getPosition(current);

           PathPlannerPath path = Paths.getPath(prevPosition, nextPosition);
           System.out.println("Path: " + path.toString());

           currentStep = new AutoStep(5, path);
           autonomousSteps[i] = currentStep;
           previous = steps[i];
           
        }        

        return autonomousSteps;
    }

    private static boolean isWait(String s) {
        return s.startsWith("w");
    }
}
