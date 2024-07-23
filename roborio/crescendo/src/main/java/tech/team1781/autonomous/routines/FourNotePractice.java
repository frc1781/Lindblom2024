package tech.team1781.autonomous.routines;

import tech.team1781.Paths;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.Positions;
import tech.team1781.autonomous.AutonomousHandler.AutoRoutine;
import tech.team1781.control.ControlSystem.Action;
import tech.team1781.utils.EVector;

public class FourNotePractice implements AutoRoutine {

    @Override
    public String getName() {
        return "23: FourNotePractice";
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
                //new AutoStep(1, Action.OFF_KICKSTAND),
                new AutoStep(6, Action.SHOOT_SUBWOOFER),
                new AutoStep(2,Action.COLLECT_RAMP),
                new AutoStep(1.5, Action.COLLECT_RAMP, Paths.getPathFromName("p2;n2"), true),
                new AutoStep(1.5, Action.COLLECT_RAMP, Paths.getPathFromName("n2;p2")),
                new AutoStep(2, Action.SHOOT_SUBWOOFER),
                new AutoStep(1.5, Action.COLLECT_RAMP, Paths.getPathFromName("p2;n3"), true),
                new AutoStep(1.5, Action.COLLECT_RAMP, Paths.getPathFromName("n3;p2")),
                new AutoStep(2, Action.SHOOT_SUBWOOFER),
                new AutoStep(1.5, Action.COLLECT_RAMP, Paths.getPathFromName("p2;n1"), true),
                new AutoStep(2, Action.COLLECT_RAMP, Paths.getPathFromName("n1;p2")),
                new AutoStep(2, Action.SHOOT_SUBWOOFER)
        };
    }

}
