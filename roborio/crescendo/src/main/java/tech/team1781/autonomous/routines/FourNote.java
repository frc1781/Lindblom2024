package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
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
                new AutoStep(6, Action.SHOOT_SUBWOOFER_NO_AIM),
                new AutoStep(5, Action.COLLECT_RAMP, Paths.getPathFromName("p2;n2")),
                new AutoStep(2, Action.SHOOT_NOTE_TWO),
                new AutoStep(5, Action.COLLECT_RAMP, Paths.getPathFromName("n2;n3")),
                new AutoStep(2, Action.SHOOT_NOTE_THREE),
                new AutoStep(5, Action.COLLECT_RAMP, Paths.getPathFromName("n3;n1")),
                new AutoStep(2, Action.SHOOT_NOTE_ONE)
        };
    }

}
