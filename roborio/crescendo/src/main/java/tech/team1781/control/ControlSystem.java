package tech.team1781.control;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.subsystems.DriveSystem;
import tech.team1781.subsystems.Scollector;
import tech.team1781.subsystems.Subsystem;
import tech.team1781.subsystems.DriveSystem.DriveSystemState;
import tech.team1781.subsystems.Scollector.ScollectorState;
import tech.team1781.subsystems.Subsystem.OperatingMode;
import tech.team1781.subsystems.Subsystem.SubsystemState;
import tech.team1781.utils.EVector;

public class ControlSystem {
    private HashMap<Action, SubsystemSetting[]> mActions = new HashMap<Action, SubsystemSetting[]>();
    private SubsystemSetting[] mCurrentSettings;
    private AutoStep mCurrentStep;
    private boolean mIsRunningAction = true;
    private Timer mStepTime;

    private ArrayList<Subsystem> mSubsystems;
    private DriveSystem mDriveSystem;
    private Scollector mScollector;

    private OperatingMode mCurrentOperatingMode;

    public enum Action {
        EXAMPLE_ACTION,
        TEST_ACTION
    }

    public ControlSystem() {
        mDriveSystem = new DriveSystem();
        mScollector = new Scollector();

        mSubsystems = new ArrayList<>();
        mSubsystems.add(mDriveSystem);
        mSubsystems.add(mScollector);

        initActions();

        mStepTime = new Timer();
    }

    public void driveChassis(EVector translation, EVector rotation) {
        // mDriveSystem.driveRaw(translation.y, translation.x, rotation.x);
        mDriveSystem.driveRaw(0.1, 0, 0);
    }

    public void setAction(Action desiredAction) {
        setAutoStep(desiredAction, null, null);
    }

    public void setAutoStep(Action desiredAction, EVector position, Trajectory trajectory) {
        mStepTime.reset();
        mStepTime.start();
        if (desiredAction != null) {
            System.out.println(desiredAction.toString());
            mCurrentSettings = mActions.get(desiredAction);
            mIsRunningAction = true;
            for (SubsystemSetting setting : mCurrentSettings) {
                setting.setState();
            }

        }

        if (position != null) {
            mDriveSystem.setPosition(position);
            mDriveSystem.setDesiredState(DriveSystemState.DRIVE_SETPOINT);
        } else if (trajectory != null) {
            mDriveSystem.setTrajectory(trajectory);
            mDriveSystem.setDesiredState(DriveSystemState.DRIVE_TRAJECTORY);
        } else {
            mDriveSystem.setDesiredState(DriveSystemState.DRIVE_MANUAL);
        }

    }

    public boolean stepIsFinished() {
        return !mIsRunningAction;
    }

    public void init(OperatingMode operatingMode) {
        mCurrentOperatingMode = operatingMode;
        for (Subsystem subsystem : mSubsystems) {
            subsystem.setOperatingMode(operatingMode);
        }
    }

    public void run() {
        for (Subsystem subsystem : mSubsystems) {
            subsystem.getToState();
            subsystem.feedStateTime(mStepTime.get());
            runSubsystemPeriodic(subsystem);
        }

        checkForRunningAction();

    }

    public boolean isRunningAction() {
        return mIsRunningAction;
    }

    public void interruptAction() {
        mIsRunningAction = false;
    }

    private void checkForRunningAction() {
        if (!mIsRunningAction) {
            return;
        }
        boolean hasUnfinishedSubsystem = false;

        for (SubsystemSetting setting : mCurrentSettings) {
            if (!setting.isFinished()) {
                hasUnfinishedSubsystem = true;
            }
        }

        mIsRunningAction = hasUnfinishedSubsystem;
    }

    private void initActions() {
        // Example below, this will never be an aciton pretty much
        defineAction(Action.EXAMPLE_ACTION,
                new SubsystemSetting(mScollector, ScollectorState.COLLECT));

        defineAction(Action.TEST_ACTION,
                new SubsystemSetting(mScollector, ScollectorState.AUTO_AIM_SHOOT));
    }

    private void defineAction(Action action, SubsystemSetting... settings) {
        mActions.put(action, settings);
    }

    private void runSubsystemPeriodic(Subsystem subsystem) {
        subsystem.genericPeriodic();
        switch (mCurrentOperatingMode) {
            case AUTONOMOUS:
                subsystem.autonomousPeriodic();
                break;
            case TELEOP:
                subsystem.teleopPeriodic();
                break;
            default:
                break;
        }

        System.out.println();
    }

}

class SubsystemSetting {
    private Subsystem mSubsystem;
    private SubsystemState mState;

    public SubsystemSetting(Subsystem subsystem, SubsystemState state) {
        mSubsystem = subsystem;
        mState = state;
    }

    public void setState() {
        mSubsystem.setDesiredState(mState);
    }

    public boolean isFinished() {
        boolean ret_val = mSubsystem.matchesDesiredState();

        return ret_val;
    }
}
