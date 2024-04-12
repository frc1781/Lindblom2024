package tech.team1781.autonomous;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import tech.team1781.ConfigMap;
import tech.team1781.control.ControlSystem;
import tech.team1781.subsystems.Subsystem;
import tech.team1781.utils.NetworkLogger;

public class AutonomousHandler {
    private SendableChooser<AutoRoutine> mAutoChooser = new SendableChooser<>();
    private AutoRoutine mSelectedRoutine;
    private ControlSystem mControlSystem;
    private int mStepIndex = 0;
    private Timer mTimer = new Timer();

    private AutoStep sampledStep;
    private AutoStep[] mSampledSteps;

    // private GenericEntry mAutoStepEntry = ConfigMap.LOG_TAB.add("Autonomous/Auto Step", "EMPTY").getEntry();
    // private GenericEntry mEndConditionEntry = ConfigMap.LOG_TAB.add("End Condition", "EMPTY").getEntry();
    // private GenericEntry mStepTimeEntry = ConfigMap.LOG_TAB.add("Step Time", 0.0).getEntry();

    public AutonomousHandler(ControlSystem controlSystem, AutoRoutine... routines) {
        mAutoChooser.setDefaultOption(routines[0].getName(), routines[0]);
        for (AutoRoutine routine : routines) {
            mAutoChooser.addOption(routine.getName(), routine);
        }
        
        ConfigMap.AUTONOMOUS_TAB.add(mAutoChooser).withSize(2, 1);

        mControlSystem = controlSystem;

        NetworkLogger.initLog("Auto Step", "EMPTY");
        NetworkLogger.initLog("End Condition", "EMPTY");
        NetworkLogger.initLog("Time", 0.0);
    }

    public void init() {
        mTimer.reset();
        mTimer.start();
        mStepIndex = 0;
        mSelectedRoutine = mAutoChooser.getSelected();
        mSampledSteps = mAutoChooser.getSelected().getSteps();

        System.out.println("THIS IS THE AUTO NAME: " + mSelectedRoutine.getName());

        sampledStep = mSelectedRoutine.getSteps()[0];
        startStep(sampledStep);

        System.out.println("initing autohandler............");
    }

    public void run() throws RoutineOverException {
        // mStepTimeEntry.setDouble(mTimer.get());
        NetworkLogger.logData("Time", mTimer.get());

        try {
            boolean controlSystemFinished = mControlSystem.stepIsFinished();
            boolean timerFinished = mTimer.get() > sampledStep.getMaxTime();
            boolean stepFinished = controlSystemFinished || timerFinished; 

            if (stepFinished) {
                // mEndConditionEntry.setString(controlSystemFinished ? "Control System Finished" : "Timer Finished");
                NetworkLogger.logData("End Condition", controlSystemFinished ? "Control System Finished" : "Timer Finished");
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
        NetworkLogger.logData("Auto Step", "Step: [" + mStepIndex + "]: " + step.toString());
        System.out.println("new step! " + step.toString());
        System.out.println(step.toString() + " ==================================================================== " + mStepIndex);
        // mControlSystem.setAutoStep(step.getAction(), step.getPosition(), step.getPath());
        mControlSystem.setAutoStep(step);
    }

    public interface AutoRoutine {

        public String getName();

        public AutoStep[] getSteps();
    }
}
