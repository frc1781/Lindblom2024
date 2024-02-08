package tech.team1781.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;

import tech.team1781.Paths;

public class AutonomousBuilder {

    public static AutoStep[] buildFromString(String s) {
        String[] steps = s.split(",");
        System.out.println("aaaaaaa: " + s);
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
           System.out.println(prevPosition.toString() + " :: " + nextPosition.toString());

           PathPlannerPath path = Paths.getPath(prevPosition, nextPosition);

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
