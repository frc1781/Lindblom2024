package tech.team1781.autonomous;

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

    public AutonomousHandler(ControlSystem controlSystem, AutoRoutine... routines) {
        mAutoChooser.setDefaultOption(routines[0].getName(), routines[0]);
        for (AutoRoutine routine : routines) {
            mAutoChooser.addOption(routine.getName(), routine);
        }
        
        ConfigMap.SHUFFLEBOARD_TAB.add(mAutoChooser);

        mControlSystem = controlSystem;
    }

    public void init() {
        mTimer.reset();
        mTimer.start();
        mStepIndex = 0;
        mSelectedRoutine = mAutoChooser.getSelected();

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
                sampledStep = mSelectedRoutine.getSteps()[mStepIndex];
                startStep(sampledStep);
            }

        } catch (Exception e) {
            mControlSystem.interruptAction();
            throw new RoutineOverException(mSelectedRoutine.getName());
        }
    }

    private void startStep(AutoStep step) {
        mControlSystem.setAutoStep(step.getAction(), step.getPosition(), step.getPath());
    }

    public interface AutoRoutine {

        public String getName();

        public AutoStep[] getSteps();
    }
}
