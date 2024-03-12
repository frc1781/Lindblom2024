package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class P2N2N3Subwoofer implements AutoRoutine{

    @Override
    public String getName() {
        return "P2N2N3Subwoofer";
    }

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
            new AutoStep(100, Action.OFF_KICKSTAND, Positions.P2),
            new AutoStep(100, Action.COLLECT_RAMP),
            new AutoStep(6, Action.SHOOT_SUBWOOFER),
            new AutoStep(3, Action.COLLECT_RAMP, Positions.N2.withZ((320/180) * Math.PI),true), 
            new AutoStep(2, Action.COLLECT_RAMP, Positions.P2),
            new AutoStep(2, Action.AUTO_AIM_SHOOT),
            new AutoStep(3, Action.COLLECT_RAMP, Positions.N3,true), 
            new AutoStep(2, Action.COLLECT_RAMP, Positions.P2),
            new AutoStep(2, Action.AUTO_AIM_SHOOT),
        };
    }
    
}
