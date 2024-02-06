package tech.team1781.control;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import tech.team1781.ConfigMap;
import tech.team1781.DriverInput.ControllerSide;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.subsystems.Climber;
import tech.team1781.subsystems.DriveSystem;
import tech.team1781.subsystems.Scollector;
import tech.team1781.subsystems.Arm;
import tech.team1781.subsystems.Subsystem;
import tech.team1781.subsystems.Arm.ArmState;
import tech.team1781.subsystems.Climber.ClimberState;
import tech.team1781.subsystems.DriveSystem.DriveSystemState;
import tech.team1781.subsystems.Scollector.ScollectorState;
import tech.team1781.subsystems.Subsystem.OperatingMode;
import tech.team1781.subsystems.Subsystem.SubsystemState;
import tech.team1781.utils.EVector;
import tech.team1781.DriverInput;
import tech.team1781.utils.LimelightHelper;

public class ControlSystem {
    private HashMap<Action, SubsystemSetting[]> mActions = new HashMap<Action, SubsystemSetting[]>();
    private SubsystemSetting[] mCurrentSettings;
    private AutoStep mCurrentStep;
    private Timer mStepTime;

    private ArrayList<Subsystem> mSubsystems;
    //private DriveSystem mDriveSystem;
    //private Scollector mScollector;
    private Climber mClimber;
   // private Arm mArm;

    private OperatingMode mCurrentOperatingMode;

    // Slew Rate Limiters for controls
    private final SlewRateLimiter mXDriveLimiter = new SlewRateLimiter(ConfigMap.DRIVER_TRANSLATION_RATE_LIMIT);
    private final SlewRateLimiter mYDriveLimiter = new SlewRateLimiter(ConfigMap.DRIVER_TRANSLATION_RATE_LIMIT);
    private final SlewRateLimiter mRotDriveLimiter = new SlewRateLimiter(ConfigMap.DRIVER_ROTATION_RATE_LIMIT);
    private final ProfiledPIDController mLimelightAimController = new ProfiledPIDController(0.075, 0, 0, new TrapezoidProfile.Constraints(1, 0.5));

    private boolean mAutoAiming = false;
    private double aimingAngle = 0.0;

    private boolean mDriverNoteManipulation = false;

    public enum Action {
        COLLECT,
        SHOOT,
        COLLECT_RAMP
    }

    public ControlSystem() {
        //mDriveSystem = new DriveSystem();
        //mScollector = new Scollector();
        mClimber = new Climber();
        //mArm = new Arm();

        mSubsystems = new ArrayList<>();
       // mSubsystems.add(mDriveSystem);
        //mSubsystems.add(mScollector);
        mSubsystems.add(mClimber);
       // mSubsystems.add(mArm);

        initActions();

        mStepTime = new Timer();
    }

    public void driverDriving(EVector translation, EVector rotation) {
        // forward and backwards
        double xVelocity = -translation.y;
        // left and right
        double yVelocity = -translation.x;
        // rotation
        double rotVelocity = -rotation.x * 0.5;

        //mDriveSystem.driveRaw(
                //mXDriveLimiter.calculate(xVelocity) * ConfigMap.MAX_VELOCITY_METERS_PER_SECOND,
               // mYDriveLimiter.calculate(yVelocity) * ConfigMap.MAX_VELOCITY_METERS_PER_SECOND,
               // mAutoAiming ? aimingAngle
                      //  : (mRotDriveLimiter.calculate(rotVelocity) * ConfigMap.MAX_VELOCITY_RADIANS_PER_SECOND));
    }


    public void setCollecting() {
        //mScollector.setDesiredState(ScollectorState.COLLECT);
       // mArm.setDesiredState(mScollector.hasNote() ? ArmState.SUBWOOFER : ArmState.COLLECT);
        mDriverNoteManipulation = true;
    }

    public void setSpit() {
       // mScollector.setDesiredState(ScollectorState.SPIT);
        mDriverNoteManipulation = true;
    }

    public void setShooting() {
       // mScollector.setDesiredState(ScollectorState.SHOOT);
        // mArm.setDesiredState(ArmState.SUBWOOFER);
        mDriverNoteManipulation = true;
    }

    public void setArmState(ArmState desiredState) {
        //mArm.setDesiredState(desiredState);
    }

    public void zeroNavX() {
       // mDriveSystem.zeroNavX();
    }

    public void centerOnAprilTag(boolean isHeld) {
        if (isHeld) {
            double x = LimelightHelper.getXOffsetOfPreferredTarget(4);
            aimingAngle = mLimelightAimController.calculate(x, 0);
        }

        mAutoAiming = isHeld;
    }

    public void setAction(Action desiredAction) {
        setAutoStep(desiredAction, null, null);
    }

    public void setAutoStep(Action desiredAction, EVector position, PathPlannerPath path) {
        mStepTime.reset();
        mStepTime.start();

        if (desiredAction != null) {
            System.out.println(desiredAction.toString());
            mCurrentSettings = mActions.get(desiredAction);
            for (SubsystemSetting setting : mCurrentSettings) {
                setting.setState();
            }

        } else {
            mCurrentSettings = null;
        }

        if (position != null) {
           // mDriveSystem.setPosition(position);
           // mDriveSystem.setDesiredState(DriveSystemState.DRIVE_SETPOINT);
       // } else if (path != null) {
            //mDriveSystem.setTrajectoryFromPath(path);
           // mDriveSystem.setDesiredState(DriveSystemState.DRIVE_TRAJECTORY);
       // } else {
           // mDriveSystem.setDesiredState(DriveSystemState.DRIVE_MANUAL);
        }

        //System.out.println(mDriveSystem.getName() + " :: " + mDriveSystem.getState().toString() + " || "
               // + mScollector.getName() + " :: " + mScollector.getState().toString() + " || " + mClimber.getName()
               // + " :: " + mClimber.getState().toString());
    }

    //public boolean stepIsFinished() {
        //return !isRunningAction() && mDriveSystem.matchesDesiredState();
   // }

    public void init(OperatingMode operatingMode) {
        mCurrentOperatingMode = operatingMode;
        for (Subsystem subsystem : mSubsystems) {
            subsystem.setOperatingMode(operatingMode);
        }

        interruptAction();

        switch (operatingMode) {
            case TELEOP:
                mXDriveLimiter.reset(0);
                mYDriveLimiter.reset(0);
                mRotDriveLimiter.reset(0);

                //mDriveSystem.setDesiredState(DriveSystem.DriveSystemState.DRIVE_MANUAL);

                mDriverNoteManipulation = false;
                break;
            case AUTONOMOUS:
                break;
            default:
                break;
        }
    }

    public void run(DriverInput driverInput) {
        //mScollector.setArmReadyToShoot(mArm.matchesDesiredState());

        switch (mCurrentOperatingMode) {
            case TELEOP:
            if (driverInput.getButton(0, "X")) {
                mClimber.setDesiredState(ClimberState.EXTEND);
            } else if(driverInput.getButton(0, "A")){
                mClimber.setDesiredState(ClimberState.RETRACT);
            } else {
                mClimber.setDesiredState(ClimberState.IDLE);
            }

                //driverDriving(
                        //driverInput.getControllerJoyAxis(ControllerSide.LEFT, ConfigMap.DRIVER_CONTROLLER_PORT),
                        //driverInput.getControllerJoyAxis(ControllerSide.RIGHT, ConfigMap.DRIVER_CONTROLLER_PORT));

                //mArm.driveManual(driverInput.getTriggerAxis(ConfigMap.DRIVER_CONTROLLER_PORT).x
                       // - driverInput.getTriggerAxis(ConfigMap.DRIVER_CONTROLLER_PORT).y);

                if (!mDriverNoteManipulation) {
                   // mScollector.setDesiredState(ScollectorState.IDLE);
                }

                mDriverNoteManipulation = false;
                break;
            case AUTONOMOUS:
               // System.out.println(mScollector.getState().toString());
               // if (mScollector.getState() == ScollectorState.COLLECT
                 //       || mScollector.getState() == ScollectorState.COLLECT_RAMP) {
                 //   if (mScollector.hasNote()) {
                  //      mArm.setDesiredState(ArmState.SUBWOOFER);
                  //  } else {
                  //      mArm.setDesiredState(ArmState.COLLECT);
                    //}
               // }
                break;
            default:
                break;
        }

        for (Subsystem subsystem : mSubsystems) {
            subsystem.getToState();
            subsystem.feedStateTime(mStepTime.get());
            runSubsystemPeriodic(subsystem);
        }
    }

    public boolean isRunningAction() {
        try {
            boolean hasUnfinishedSubsystem = false;
            for (SubsystemSetting setting : mCurrentSettings) {
                if (!setting.isFinished()) {
                    hasUnfinishedSubsystem = true;
                }
            }

            return hasUnfinishedSubsystem;
        } catch (NullPointerException e) {
            return false;
        }
    }

    public void interruptAction() {
        mCurrentSettings = null;

        for (Subsystem s : mSubsystems) {
            s.restoreDefault();
        }
    }

    private void initActions() {
       // defineAction(Action.COLLECT,
                //new SubsystemSetting(mScollector, ScollectorState.COLLECT),
                //new SubsystemSetting(mArm, ArmState.COLLECT));

       // defineAction(Action.SHOOT,
                //new SubsystemSetting(mScollector, ScollectorState.SHOOT),
               // new SubsystemSetting(mArm, ArmState.SUBWOOFER));

       // defineAction(Action.COLLECT_RAMP,
                //new SubsystemSetting(mScollector, ScollectorState.COLLECT_RAMP),
               // new SubsystemSetting(mArm, ArmState.COLLECT));
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
