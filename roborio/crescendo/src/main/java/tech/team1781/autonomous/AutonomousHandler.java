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

    public AutonomousHandler(AutoRoutine... routines) {
        mAutoChooser.setDefaultOption(routines[0].getName(), routines[0]);
        for (AutoRoutine routine : routines) {
            mAutoChooser.addOption(routine.getName(), routine);
        }

        ConfigMap.AUTONOMOUS_TAB.add(mAutoChooser).withSize(2, 1);
        Logger.recordOutput("Autonomous/ChosenRoutine", mAutoChooser.getSelected().getName());

        Logger.recordOutput("Autonomous/CurrentStep", "None");
        Logger.recordOutput("Autonomous/EndCodition", "None");
        Logger.recordOutput("Autonomous/Time", 0.0);
    }

    public void setControlSystem(ControlSystem controlSystem) {
        mControlSystem = controlSystem;
    }

    public void init() {
        mTimer.reset();
        mStepIndex = 0;
        mSelectedRoutine = mAutoChooser.getSelected();
        mSampledSteps = mSelectedRoutine.getSteps();
        sampledStep = mSelectedRoutine.getSteps()[0];

        Logger.recordOutput("Autonomous/Routine",  mSelectedRoutine.getName());
        Logger.recordOutput("Autonomous/ChosenRoutine", mAutoChooser.getSelected().getName());
        System.out.println("initing autohandler............");
    }

    public void run() throws RoutineOverException {
        Logger.recordOutput("Autonomous/Timer", mTimer.get());
        Logger.recordOutput("Autonomous/StepIndex", mStepIndex);

        try {
            boolean controlSystemFinished = mControlSystem.stepIsFinished();
            boolean timerFinished = mTimer.get() > sampledStep.getMaxTime();
            boolean stepFinished = controlSystemFinished || timerFinished;

            if (stepFinished || mStepIndex == 0) {
                Logger.recordOutput("Autonomous/EndCondition",controlSystemFinished ? "Control System Finished" : "Timer Finished" );
                mTimer.reset();
                mTimer.start();
                sampledStep = mSampledSteps[mStepIndex];
                startStep(sampledStep);

                mStepIndex++;
            }

        } catch (Exception e) {
            System.out.println(e);
            mControlSystem.interruptAction();
            
            throw new RoutineOverException(mSelectedRoutine.getName());
        }
    }

    private void startStep(AutoStep step) {
        // mAutoStepEntry.setString("Step: [" + mStepIndex + "]: " + step.toString());
        Logger.recordOutput("Autonomous/AutoStep", "Step: [" + mStepIndex + "]: " + step.toString());
        System.out.println("new step! " + step.toString());
        System.out.println(step.toString() + " ==================================================================== "
                + mStepIndex);
        mControlSystem.setAutoStep(step);
    }

    public AutoRoutine getAutoRoutine() {
        return mSelectedRoutine;
    }

    public Pose2d getStartPosition() throws NoAutoRoutineException {
        if (mSelectedRoutine != null) {
            for (int i = 0; i < mSampledSteps.length; i++) {
                System.out.println(i);
                switch (mSampledSteps[i].getType()) {
                    case POSITION:
                    case POSITION_AND_ACTION:
                        return mSampledSteps[i].getWaypointHolder().getPosition().toPose2d();
                    case PATH_AND_ACTION:
                    case PATH:
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
        } else {
            System.out.println("selected routine in null");
        }

        throw new NoAutoRoutineException();
    }

    public class NoAutoRoutineException extends Exception {
        @Override 
        public void printStackTrace() {
         System.out.printf("The routine was null or invalid. We should wait till we being auto.");
        }
    } 

    public interface AutoRoutine {

        public String getName();

        public AutoStep[] getSteps();
    }
}
