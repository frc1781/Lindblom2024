package tech.team1781.control;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import tech.team1781.ConfigMap;
import tech.team1781.DriverInput.ControllerSide;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.subsystems.DriveSystem;
import tech.team1781.subsystems.Subsystem;
import tech.team1781.subsystems.DriveSystem.DriveSystemState;
import tech.team1781.subsystems.Subsystem.OperatingMode;
import tech.team1781.subsystems.Subsystem.SubsystemState;
import tech.team1781.utils.EVector;
import tech.team1781.DriverInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;

public class ControlSystem {
    private HashMap<Action, SubsystemSetting[]> mActions = new HashMap<Action, SubsystemSetting[]>();
    private SubsystemSetting[] mCurrentSettings;
    private AutoStep mCurrentStep;
    private boolean mIsRunningAction = false;
    private Timer mStepTime;
    private final CANSparkMax shooterMotorLeft;
    private final CANSparkMax shooterMotorRight;
    private final TalonFX collectorMotor;
    private ArrayList<Subsystem> mSubsystems;
    private DriveSystem mDriveSystem;
    
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

        mSubsystems = new ArrayList<>();
        mSubsystems.add(mDriveSystem);

        initActions();

        mStepTime = new Timer();
        shooterMotorLeft = new CANSparkMax(30, MotorType.kBrushless);
        shooterMotorRight = new CANSparkMax(5, MotorType.kBrushless);
        collectorMotor = new TalonFX(10);
    }

    public void driverDriving(EVector translation, EVector rotation) {
        //forward and backwards
        double xVelocity = -translation.y;
        //left and right
        double yVelocity = -translation.x;
        //rotation
        double rotVelocity = -rotation.x;

        shooterMotorLeft.set(-translation.y);
        shooterMotorRight.set(-translation.y);
        collectorMotor.set(-rotation.y);
        
//        mDriveSystem.driveRaw(
//            mXDriveLimiter.calculate(xVelocity) * ConfigMap.MAX_VELOCITY_METERS_PER_SECOND, 
//            mYDriveLimiter.calculate(yVelocity) * ConfigMap.MAX_VELOCITY_METERS_PER_SECOND, 
//            mRotDriveLimiter.calculate(rotVelocity) * ConfigMap.MAX_VELOCITY_RADIANS_PER_SECOND);
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

    public void run(DriverInput driverInput) {
        driverDriving(
            driverInput.getControllerJoyAxis(ControllerSide.LEFT, ConfigMap.DRIVER_CONTROLLER_PORT), 
            driverInput.getControllerJoyAxis(ControllerSide.RIGHT, ConfigMap.DRIVER_CONTROLLER_PORT)
        );

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
