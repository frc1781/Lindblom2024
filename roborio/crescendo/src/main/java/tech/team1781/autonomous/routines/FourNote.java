package tech.team1781.autonomous.routines;

import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class FourNote implements AutoRoutine {

    @Override
    public String getName() {
        return "13: FourNote";
    }

    // 1. Shoot note at p2 without zeroing on collect, see how fast we can shoot
    // even if not at speed. Should begin ramping
    // shooter BEFORE knocking off kickstand.
    // 2. Move out to collect n2 and shoot without returning to subwoofer,
    // as soon as collect there is no movement, just a shoot when angle of arm is
    // ready.
    // 3. Collect n1 and return to p2 to shoot at subwoofer.
    // 4. Collect n3 and shoot as soon as collected, aiming with auto aim.

    @Override
    public AutoStep[] getSteps() {
        return new AutoStep[] {
                new AutoStep(100, Action.OFF_KICKSTAND),
                new AutoStep(0.1, Positions.P2),
                new AutoStep(6, Action.SHOOT_SUBWOOFER),
                new AutoStep(3, Action.COLLECT_RAMP, Positions.N2, true),
                new AutoStep(2, Action.AUTO_AIM_SHOOT),
                new AutoStep(3, Action.COLLECT_RAMP, EVector.positionWithDegrees(1.79, 6.42, 26)),
                new AutoStep(2, Action.COLLECT_RAMP, Positions.N1, true),
                new AutoStep(2, Action.COLLECT_RAMP, Positions.P2),
                new AutoStep(6, Action.SHOOT_SUBWOOFER),
                new AutoStep(1.5, Action.COLLECT_RAMP, EVector.positionWithDegrees(2.0, 4.1, 0)),
                new AutoStep(3, Action.COLLECT_RAMP_STAY_DOWN, Positions.N3, true),
                new AutoStep(1.5, Action.COLLECT_RAMP, EVector.positionWithDegrees(2.0, 4.1, 308)),
                // new AutoStep(2, Action.COLLECT_RAMP, Positions.P2),
                new AutoStep(5, Action.SHOOT_NOTE_THREE),
        };
    }

}
