package tech.team1781.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import org.littletonrobotics.junction.Logger;
import tech.team1781.ConfigMap;
import tech.team1781.control.ControlSystem;

import java.util.HashMap;


public class AutonomousHandler {
    private SendableChooser<AutoRoutine> mAutoChooser = new SendableChooser<>();
    private AutoRoutine mSelectedRoutine;
    private ControlSystem mControlSystem;
    private int mStepIndex = 0;
    private Timer mTimer = new Timer();
    private HashMap<Integer, PathPlannerPath> mPaths;

    private boolean pathsGeneratedForRed;


    private AutoStep sampledStep;
    private AutoStep[] mSampledSteps;

    public AutonomousHandler(AutoRoutine... routines) {
        mAutoChooser.setDefaultOption(routines[0].getName(), routines[0]);
        for (AutoRoutine routine : routines) {
            mAutoChooser.addOption(routine.getName(), routine);
        }

        mPaths = new HashMap<>();

        ConfigMap.AUTONOMOUS_TAB.add(mAutoChooser).withSize(2, 1);
        Logger.recordOutput("Autonomous/ChosenRoutine", mAutoChooser.getSelected().getName());

        Logger.recordOutput("Autonomous/CurrentStep", "None");
        Logger.recordOutput("Autonomous/EndCodition", "None");
        Logger.recordOutput("Autonomous/Timer", 0.0);
    }

    public void setControlSystem(ControlSystem controlSystem) {
        mControlSystem = controlSystem;
    }

    public void checkSelectedRoutine() {
        boolean currentAlliance = ControlSystem.isRed();
        AutoRoutine chosenRoutine = mAutoChooser.getSelected();
        Logger.recordOutput("Autonomous/ChosenRoutine", chosenRoutine.getName());

        if (mSelectedRoutine != chosenRoutine || pathsGeneratedForRed != currentAlliance) {
            mTimer.reset();
            mStepIndex = 0;
            mSelectedRoutine = mAutoChooser.getSelected();
            mSampledSteps = mSelectedRoutine.getSteps();
            sampledStep = mSelectedRoutine.getSteps()[0];

            pathsGeneratedForRed = currentAlliance;
            Logger.recordOutput("Autonomous/Routine", mSelectedRoutine.getName());
            System.out.println("Cached currently selected routine");
        }
    }

    public void init() {
        boolean currentAlliance = ControlSystem.isRed();
        mTimer.reset();
        mStepIndex = 0;

        if (mSelectedRoutine != mAutoChooser.getSelected() || pathsGeneratedForRed != currentAlliance) {
            mTimer.reset();
            mStepIndex = 0;
            mSelectedRoutine = mAutoChooser.getSelected();
            mSampledSteps = mSelectedRoutine.getSteps();
            sampledStep = mSelectedRoutine.getSteps()[0];
            System.out.println("Cached Routine not valid, reloading...");
        }

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
                Logger.recordOutput("Autonomous/EndCondition", controlSystemFinished ? "Control System Finished" : "Timer Finished" );
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
         System.out.printf("The routine was null or invalid. We should wait till we start auto.");
        }
    }

    public interface AutoRoutine {

        public String getName();

        public AutoStep[] getSteps();
    }
}
