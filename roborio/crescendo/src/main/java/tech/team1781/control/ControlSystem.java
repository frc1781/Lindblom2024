package tech.team1781.control;

import java.util.ArrayList;
import java.util.HashMap;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import tech.team1781.ConfigMap;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.subsystems.Collector;
import tech.team1781.subsystems.DriveSystem;
import tech.team1781.subsystems.Subsystem;
import tech.team1781.subsystems.DriveSystem.DriveSystemState;
import tech.team1781.subsystems.Subsystem.OperatingMode;
import tech.team1781.subsystems.Subsystem.SubsystemState;
import tech.team1781.utils.EVector;

public class ControlSystem {
    private HashMap<Action, SubsystemSetting[]> mActions = new HashMap<Action, SubsystemSetting[]>();
    private SubsystemSetting[] mCurrentSettings;
    private AutoStep mCurrentStep;
    private boolean mIsRunningAction = false;
    private Timer mStepTime;

    private ArrayList<Subsystem> mSubsystems;
    private DriveSystem mDriveSystem;
    private Collector mCollector; 
    
    private OperatingMode mCurrentOperatingMode;

    // Slew Rate Limiters for controls
    private final SlewRateLimiter mXDriveLimiter = new SlewRateLimiter(ConfigMap.DRIVER_TRANSLATION_RATE_LIMIT);
    private final SlewRateLimiter mYDriveLimiter = new SlewRateLimiter(ConfigMap.DRIVER_TRANSLATION_RATE_LIMIT);
    private final SlewRateLimiter mRotDriveLimiter = new SlewRateLimiter(ConfigMap.DRIVER_ROTATION_RATE_LIMIT);

    public enum Action {
        EXAMPLE_ACTION
    }

    public ControlSystem() {
        mDriveSystem = new DriveSystem();
        mCollector = new Collector();

        mSubsystems = new ArrayList<>();
        mSubsystems.add(mDriveSystem);
        mSubsystems.add(mCollector);

        initActions();

        mStepTime = new Timer();
    }

    public void driveChassis(EVector translation, EVector rotation) {
        mDriveSystem.drawWithMaxVelo(
            mXDriveLimiter.calculate(translation.y), 
            mYDriveLimiter.calculate(translation.x), 
            mRotDriveLimiter.calculate(rotation.x));
    }

    public void setCollector(boolean isCollecting) {
        mCollector.setDesiredState(
            isCollecting ? Collector.CollectorState.COLLECT : Collector.CollectorState.IDLE
        );
    }

    public void zeroNavX() {
        mDriveSystem.zeroNavX();
    }

    public void setAction(Action desiredAction) {
        setAutoStep(desiredAction, null, null);
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
        return !mIsRunningAction; //&& mDriveSystem.matchesDesiredState();
    }

    public void init(OperatingMode operatingMode) {
        mCurrentOperatingMode = operatingMode;
        for(Subsystem subsystem : mSubsystems) {
            subsystem.setOperatingMode(operatingMode);
        }

        switch(operatingMode) {
            case TELEOP:
                mXDriveLimiter.reset(0);
                mYDriveLimiter.reset(0);
                mRotDriveLimiter.reset(0);
            break;
            default:
            break;
        }
    }

    public void run() {
        for (Subsystem subsystem : mSubsystems) {
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
                break;
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

    private void runSubsystemPeriodic(Subsystem subsystem) {
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
        return mSubsystem.matchesDesiredState();
    }
}
