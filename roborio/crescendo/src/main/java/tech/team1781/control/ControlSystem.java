package tech.team1781.control;

import java.util.ArrayList;
import java.util.HashMap;

//TODO: change to pathplanner trajectory
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.subsystems.DriveSystemController;
import tech.team1781.subsystems.SubsystemController;
import tech.team1781.subsystems.DriveSystemController.DriveSystemState;
import tech.team1781.subsystems.SubsystemController.OperatingMode;
import tech.team1781.subsystems.SubsystemController.SubsystemState;
import tech.team1781.utils.EVector;

public class ControlSystem {
    private HashMap<Action, SubsystemSetting[]> mActions = new HashMap<Action, SubsystemSetting[]>();
    private SubsystemSetting[] mCurrentSettings;
    private AutoStep mCurrentStep;
    private boolean mIsRunningAction = false;
    private Timer mStepTime;

    private ArrayList<SubsystemController> mSubsystems;
    private DriveSystemController mDriveSystem;
    
    private OperatingMode mCurrentOperatingMode;

    public enum Action {
        EXAMPLE_ACTION
    }

    public ControlSystem() {
        mDriveSystem = new DriveSystemController();

        mSubsystems = new ArrayList<>();
        mSubsystems.add(mDriveSystem);

        initActions();

        mStepTime = new Timer();

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

    public void init(OperatingMode operatingMode) {
        for(SubsystemController subsystem : mSubsystems) {
            subsystem.setOperatingMode(operatingMode);
        }
    }

    public void run() {
        for (SubsystemController subsystem : mSubsystems) {
            subsystem.getToState();
            subsystem.feedStateTime(mStepTime.get());
            runSubsystemPeriodic(subsystem);
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

    private void runSubsystemPeriodic(SubsystemController subsystem) {
        subsystem.genericPeriodic();        
        switch(mCurrentOperatingMode) {
            case AUTONOMOUS:
                subsystem.autonomousPeriodic();
            break;
            case TELEOP:
                subsystem.teleopPeriodic();
            break;
            default: 
            break; 
                
        }
    }

}

class SubsystemSetting {
    private SubsystemController mSubsystem;
    private SubsystemState mState;

    public SubsystemSetting(SubsystemController subsystem, SubsystemState state) {
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
