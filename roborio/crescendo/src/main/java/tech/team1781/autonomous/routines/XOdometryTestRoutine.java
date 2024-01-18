package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.AutonomousHandler;
import tech.team1781.utils.EVector;

public class XOdometryTestRoutine implements AutonomousHandler.AutoRoutine {

    @Override
    public String getName() {
        return "X+ Odometry Test";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] { new AutoStep(10, new EVector(1, 0 )) };
    }
}
