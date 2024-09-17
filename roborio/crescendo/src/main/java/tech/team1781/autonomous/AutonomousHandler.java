package tech.team1781.autonomous;

import org.littletonrobotics.junction.Logger;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.concurrent.Event;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import tech.team1781.ConfigMap;
import tech.team1781.control.ControlSystem;
import tech.team1781.subsystems.Subsystem;


public class AutonomousHandler {
    private SendableChooser<AutoRoutine> mAutoChooser = new SendableChooser<>();
    private AutoRoutine mSelectedRoutine;
    private ControlSystem mControlSystem;
    private int mStepIndex = 0;
    private Timer mTimer = new Timer();

    private AutoStep sampledStep;
    private AutoStep[] mSampledSteps;

    public AutonomousHandler(ControlSystem controlSystem, AutoRoutine... routines) {
        mAutoChooser.setDefaultOption(routines[0].getName(), routines[0]);
        for (AutoRoutine routine : routines) {
            mAutoChooser.addOption(routine.getName(), routine);
        }

        ConfigMap.AUTONOMOUS_TAB.add(mAutoChooser).withSize(2, 1);

        mControlSystem = controlSystem;
        Logger.recordOutput("Autonomous/ChosenRoutine", mAutoChooser.getSelected().getName());

        Logger.recordOutput("Autonomous/CurrentStep", "None");
        Logger.recordOutput("Autonomous/EndCodition", "None");
        Logger.recordOutput("Autonomous/Time", 0.0);
    }

    public void init() {
        mTimer.reset();
        mTimer.start();
        mStepIndex = 0;
        mSelectedRoutine = mAutoChooser.getSelected();
        mSampledSteps = mAutoChooser.getSelected().getSteps();

        Logger.recordOutput("Autonomous/Routine", mSelectedRoutine.getName());

        sampledStep = mSelectedRoutine.getSteps()[0];
        startStep(sampledStep);

        System.out.println("initing autohandler............");
    }

    public void run() throws RoutineOverException {
        Logger.recordOutput("Autonomous/Timer", mTimer.get());

        try {
            boolean controlSystemFinished = mControlSystem.stepIsFinished();
            boolean timerFinished = mTimer.get() > sampledStep.getMaxTime();
            boolean stepFinished = controlSystemFinished || timerFinished;

            if (stepFinished) {
                Logger.recordOutput("Autonomus/EndCondition",controlSystemFinished ? "Control System Finished" : "Timer Finished" );
                mStepIndex++;
                mTimer.reset();
                mTimer.start();
                sampledStep = mSampledSteps[mStepIndex];
                startStep(sampledStep);
            }

        } catch (Exception e) {
            mControlSystem.interruptAction();
            throw new RoutineOverException(mSelectedRoutine.getName());
        }
    }

    private void startStep(AutoStep step) {
        // mAutoStepEntry.setString("Step: [" + mStepIndex + "]: " + step.toString());
         Logger.recordOutput("Autonomus/AutoStep", "Step: [" + mStepIndex + "]: " + step.toString());
        System.out.println("new step! " + step.toString());
        System.out.println(step.toString() + " ==================================================================== "
                + mStepIndex);
        mControlSystem.setAutoStep(step);
    }

    public AutoRoutine getAutoRoutine() {
        return mSelectedRoutine;
    }

    public Pose2d getStartPosition() {
        if (mSelectedRoutine != null) {
            for (int i = 0; i < mSampledSteps.length; i++) {
                switch (mSampledSteps[i].getType()) {
                    case POSITION:
                    case POSITION_AND_ACTION:
                        return mSampledSteps[i].getWaypointHolder().getPosition().toPose2d();
                    case PATH:
                    case PATH_AND_ACTION:
                        return mSampledSteps[i].getPath().getPreviewStartingHolonomicPose();
                    case WAIT:
                    case NOTE_POSITION:
                    case ACTION:
                    case ROTATION:
                    case ROTATION_AND_ACTION:
                    default:
                        continue;
                }
            }
        }

        return new Pose2d();
    }

    public interface AutoRoutine {

        public String getName();

        public AutoStep[] getSteps();
    }
}
