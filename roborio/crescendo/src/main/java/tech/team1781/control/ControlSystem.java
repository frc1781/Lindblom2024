package tech.team1781.control;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.subsystems.DriveSystem;
import tech.team1781.subsystems.EESubsystem;
import tech.team1781.subsystems.DriveSystem.DriveSystemState;
import tech.team1781.subsystems.EESubsystem.SubsystemState;
import tech.team1781.utils.EVector;

public class ControlSystem {
    private HashMap<Action, SubsystemSetting[]> mActions = new HashMap<Action, SubsystemSetting[]>();
    private SubsystemSetting[] mCurrentSettings;
    private AutoStep mCurrentStep;
    private boolean mIsRunningAction = false;
    private Timer mStepTime;

    private ArrayList<EESubsystem> mSubsystems;
    private DriveSystem mDriveSystem;

    public enum Action {
        EXAMPLE_ACTION
    }

    public ControlSystem(DriveSystem driveSystem) {
        mDriveSystem = driveSystem;

        mSubsystems = new ArrayList<>();
        mSubsystems.add(driveSystem);

        initActions();

        mStepTime = new Timer();

<<<<<<< Updated upstream
=======
    public void driveChassis(EVector translation, EVector rotation) {
         mDriveSystem.driveRaw(translation.y, translation.x, rotation.x);
        //mDriveSystem.driveRaw(0.1, 0, 0);
    }

    public void setAction(Action desiredAction) {
        setAutoStep(desiredAction, null, null);
>>>>>>> Stashed changes
    }

    public void setAutoStep(Action desiredAction, EVector position, Trajectory trajectory) {
        mStepTime.reset();
        mStepTime.start();
        if (desiredAction != null) {
            mCurrentSettings = mActions.get(desiredAction);
            mIsRunningAction = true;
            for (SubsystemSetting setting : mCurrentSettings) {
                setting.setState();
            }

        }

        if (position != null) {
            // mDriveSystem.setPosition(position);
            mDriveSystem.setDesiredState(DriveSystemState.DRIVE_SETPOINT);
        } else if (trajectory != null) {
            // mDriveSystem.setTrajectory(trajectory);
            mDriveSystem.setDesiredState(DriveSystemState.DRIVE_TRAJECTORY);
        } else {
            mDriveSystem.setDesiredState(DriveSystemState.DRIVE_MANUAL);
        }

    }

    public boolean stepIsFinished() {
        return !mIsRunningAction; //&& mDriveSystem.matchesDesiredState();
    }

    public void run() {
        for (EESubsystem subsystem : mSubsystems) {
            subsystem.getToState();
            subsystem.feedStateTime(mStepTime.get());
        }

        if (!mIsRunningAction) {
            return;
        }

        boolean hasUnfinishedSubsystem = false;

        for (SubsystemSetting setting : mCurrentSettings) {
            if (!setting.isFinished()) {
                hasUnfinishedSubsystem = true;
            }
        }

        mIsRunningAction = !hasUnfinishedSubsystem;
    }

    public boolean isRunningAction() {
        return mIsRunningAction;
    }

    public void interruptAction() {
        mIsRunningAction = false;
    }

    private void initActions() {
        //Example below, this will never be an aciton pretty much 
        defineAction(Action.EXAMPLE_ACTION, 
            new SubsystemSetting(mDriveSystem, DriveSystemState.DRIVE_TRAJECTORY));
    }

    private void defineAction(Action action, SubsystemSetting... settings) {
        mActions.put(action, settings);
    }

}

class SubsystemSetting {
    private EESubsystem mSubsystem;
    private SubsystemState mState;

    public SubsystemSetting(EESubsystem subsystem, SubsystemState state) {
        mSubsystem = subsystem;
        mState = state;
    }

    public void setState() {
        mSubsystem.setDesiredState(mState);
    }

    public boolean isFinished() {
        return mSubsystem.matchesDesiredState();
    }
}
