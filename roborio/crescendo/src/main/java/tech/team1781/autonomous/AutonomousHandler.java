package tech.team1781.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import tech.team1781.ConfigMap;
import tech.team1781.autonomous.routines.FourNoteRoutine;
import tech.team1781.autonomous.routines.PIDTuningRoutine;
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
        
        ConfigMap.AUTONOMOUS_TAB.add(mAutoChooser);

        mControlSystem = controlSystem;
    }

    public void init() {
        mTimer.reset();
        mTimer.start();
        mStepIndex = 0;
        mSelectedRoutine = mAutoChooser.getSelected();
        mSampledSteps = mAutoChooser.getSelected().getSteps();


        sampledStep = mSelectedRoutine.getSteps()[0];
        startStep(sampledStep);

        System.out.println("initing autohandler............");
    }

    public void run() throws RoutineOverException {
        try {

            boolean stepFinished = mControlSystem.stepIsFinished() ||
                    mTimer.get() > sampledStep.getMaxTime();

            // System.out.println(mStepIndex + "::" + mControlSystem.stepIsFinished() + " :: " + (mTimer.get() > sampledStep.getMaxTime()));
            if (stepFinished) {
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
        System.out.println("new step!");
        System.out.println(step.toString() + " ==================================================================== " + mStepIndex);
        mControlSystem.setAutoStep(step.getAction(), step.getPosition(), step.getPath());
    }

    public interface AutoRoutine {

        public String getName();

        public AutoStep[] getSteps();
    }
}
