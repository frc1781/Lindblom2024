package tech.team1781.control;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import tech.team1781.ConfigMap;
import tech.team1781.DriverInput.ControllerSide;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.subsystems.Climber;
import tech.team1781.subsystems.DriveSystem;
import tech.team1781.subsystems.Scollector;
import tech.team1781.subsystems.Arm;
import tech.team1781.subsystems.Subsystem;
import tech.team1781.subsystems.Arm.ArmState;
import tech.team1781.subsystems.DriveSystem.DriveSystemState;
import tech.team1781.subsystems.Scollector.ScollectorState;
import tech.team1781.subsystems.Subsystem.OperatingMode;
import tech.team1781.subsystems.Subsystem.SubsystemState;
import tech.team1781.utils.EVector;
import tech.team1781.DriverInput;
import tech.team1781.ShuffleboardStyle;
import tech.team1781.utils.LimelightHelper;

public class ControlSystem {
    private HashMap<Action, SubsystemSetting[]> mActions = new HashMap<Action, SubsystemSetting[]>();
    private SubsystemSetting[] mCurrentSettings;
    private AutoStep mCurrentStep;
    private Timer mStepTime;

    private ArrayList<Subsystem> mSubsystems;
    private DriveSystem mDriveSystem;
    private Scollector mScollector;
    private Climber mClimber;
    private Arm mArm;

    private OperatingMode mCurrentOperatingMode;

    // Slew Rate Limiters for controls
    private final SlewRateLimiter mXDriveLimiter = new SlewRateLimiter(ConfigMap.DRIVER_TRANSLATION_RATE_LIMIT);
    private final SlewRateLimiter mYDriveLimiter = new SlewRateLimiter(ConfigMap.DRIVER_TRANSLATION_RATE_LIMIT);
    private final SlewRateLimiter mRotDriveLimiter = new SlewRateLimiter(ConfigMap.DRIVER_ROTATION_RATE_LIMIT);
    private final ProfiledPIDController mLimelightAimController = new ProfiledPIDController(0.075, 0, 0,
            new TrapezoidProfile.Constraints(1, 0.5));

    private boolean mAutoAiming = false;
    private double aimingAngle = 0.0;

    // CURRENT STATE OF INPUT for HOLD DOWN BUTTONS
    private boolean mKeepArmDownButton = false;
    private boolean mCollectingButton = false;
    private boolean mPrepareToShootButton = false;
    private boolean mShootButton = false;
    private boolean mSpitButton = false;
    private boolean mClimberRetractButton = false;
    private boolean mClimberExtendButton = false;
    private boolean mCollectingHighButton = false;
    private boolean mAmpButton = false;


    private NetworkTable mBackLimelightTable = NetworkTableInstance.getDefault().getTable(ConfigMap.BACK_LIMELIGHT_NAME);
    private GenericEntry mSeesAprilTagEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Sees AprilTag", false, ShuffleboardStyle.SEES_APRILTAG);

    public enum Action {
        COLLECT,
        SHOOT,
        COLLECT_RAMP,
        COLLECT_AUTO_SHOOT,
        SYSID
    }

    public ControlSystem() {
        mDriveSystem = new DriveSystem();
        mScollector = new Scollector();
        mClimber = new Climber();
        mArm = new Arm();

        mSubsystems = new ArrayList<>();
        mSubsystems.add(mDriveSystem);
        mSubsystems.add(mScollector);
        mSubsystems.add(mClimber);
        mSubsystems.add(mArm);

        initActions();

        mStepTime = new Timer();
    }

    public static boolean isRed() {
        return DriverStation.getAlliance().get() == Alliance.Red;
    }

    public void driverDriving(EVector translation, EVector rotation, EVector triggers) {
        boolean isRed = DriverStation.getAlliance().get() == Alliance.Red;
        int mult = isRed ? -1 : 1;
        final double triggermult = 0.1;

        // forward and backwards
        double xVelocity = -translation.y * mult;
        // left and right
        double yVelocity = -translation.x * mult;
        // rotation
        double rotVelocity = -rotation.x * 0.5 + ((triggers.x * triggermult) - (triggers.y * triggermult));

        mDriveSystem.driveRaw(
                mXDriveLimiter.calculate(xVelocity) * ConfigMap.MAX_VELOCITY_METERS_PER_SECOND,
                mYDriveLimiter.calculate(yVelocity) * ConfigMap.MAX_VELOCITY_METERS_PER_SECOND,
                mAutoAiming ? aimingAngle
                        : (mRotDriveLimiter.calculate(rotVelocity) * ConfigMap.MAX_VELOCITY_RADIANS_PER_SECOND));

        odometryUpdate(xVelocity, yVelocity);
    }

    public void keepArmDown(boolean pushingKeepDown) {
        if (mKeepArmDownButton == pushingKeepDown) {
            return; // already set, do nothing
        }

        mKeepArmDownButton = pushingKeepDown;
        if (mKeepArmDownButton) {
            mArm.setDesiredState(ArmState.COLLECT);
        } else {
            if (!mPrepareToShootButton && !mCollectingButton) {
                mArm.setDesiredState(ArmState.SAFE);
            }
            if (mPrepareToShootButton) {
                mScollector.setDesiredState(ScollectorState.RAMP_SHOOTER);
                mArm.setDesiredState(ArmState.AUTO_ANGLE);
            }
        }
    }

    public void setCollecting(boolean pushingCollect) {
        if (mCollectingButton == pushingCollect) {
            return; // no change in state
        }

        mCollectingButton = pushingCollect;
        if (mCollectingButton) {
            mScollector.setDesiredState(ScollectorState.COLLECT);
            mArm.setDesiredState(ArmState.COLLECT);
        } else {
            if (!mKeepArmDownButton && !mPrepareToShootButton) {
                mArm.setDesiredState(ArmState.SAFE);
            }
            if (mPrepareToShootButton) {
                mScollector.setDesiredState(ScollectorState.RAMP_SHOOTER);

                if (!mKeepArmDownButton && !mCollectingButton) {
                    mArm.setDesiredState(ArmState.AUTO_ANGLE);
                }
            } else {
                mScollector.setDesiredState(ScollectorState.IDLE);
            }
        }
    }

    public void setSpit(boolean pushingSpit) {
        if (mSpitButton == pushingSpit) {
            return; // no change in state
        }

        mSpitButton = pushingSpit;
        if (mSpitButton) {
            mScollector.setDesiredState(ScollectorState.SPIT);
        } else {
            mScollector.setDesiredState(ScollectorState.IDLE);
        }
    }

    public void setShooting(boolean pushingShoot) {
        if (mShootButton == pushingShoot) {
            return; // no change in state
        }

        mShootButton = pushingShoot;
        if (mShootButton) {
            mScollector.setDesiredState(ScollectorState.SHOOT);
        } else if (!mKeepArmDownButton && !mCollectingButton) {
            mArm.setDesiredState(ArmState.SAFE);
            mScollector.setDesiredState(ScollectorState.IDLE);
        }
    }

    public void manualAdjustAngle(double diff) {
        mArm.manualAdjustAngle(diff);
    }

    public void calibratePosition(){
        mDriveSystem.setOdometry(getLimelightPose());
    }

    public void setPrepareToShoot(boolean pushingPrepare) {
        if (mPrepareToShootButton == pushingPrepare) {
            return; // no change in state
        }
        mPrepareToShootButton = pushingPrepare;
        if (mPrepareToShootButton) {
            if (mCollectingButton || mKeepArmDownButton) {
                mScollector.setDesiredState(ScollectorState.COLLECT_RAMP);
            } else if (!mKeepArmDownButton) {
                mScollector.setDesiredState(ScollectorState.RAMP_SHOOTER);
                mArm.setDesiredState(ArmState.AUTO_ANGLE);
            }
        } else {
            if (!mCollectingButton && !mKeepArmDownButton) {
                mArm.setDesiredState(ArmState.SAFE);
            }
            if (mCollectingButton) {
                mScollector.setDesiredState(ScollectorState.COLLECT);
            } else {
                mScollector.setDesiredState(ScollectorState.IDLE);
            }
        }
    }

    public void setClimberRetract(boolean pushingRetract) {
        if (mClimberRetractButton == pushingRetract) {
            return; // no change in state
        }

        mClimberRetractButton = pushingRetract;
        if (mClimberRetractButton) {
            mClimber.setDesiredState(Climber.ClimberState.RETRACT);
        } else if (!mClimberExtendButton) {
            mClimber.setDesiredState(Climber.ClimberState.IDLE);
        }
    }

    public void setClimberExtend(boolean pushingExtend) {
        if (mClimberExtendButton == pushingExtend) {
            return; // no change in state
        }

        mClimberExtendButton = pushingExtend;
        if (mClimberExtendButton) {
            mClimber.setDesiredState(Climber.ClimberState.EXTEND);
        } else if (!mClimberRetractButton) {
            mClimber.setDesiredState(Climber.ClimberState.IDLE);
        }
    }

    public void setCollectHigh(boolean collectingHigh) {
        if(mCollectingHighButton == collectingHigh) {
            return;
        }

        mCollectingHighButton = collectingHigh;

        if(!mCollectingButton && collectingHigh && !mKeepArmDownButton){
            mArm.setDesiredState(ArmState.COLLECT_HIGH);
            mScollector.setDesiredState(ScollectorState.COLLECT);
        }

        if(!collectingHigh && !mKeepArmDownButton && !mPrepareToShootButton && !mCollectingButton){
            mArm.setDesiredState(ArmState.SAFE);
            mScollector.setDesiredState(ScollectorState.IDLE);
        }
    }

    public void setAmp(boolean isAmp){
        if(mAmpButton == isAmp){
            return;
        }

        mAmpButton = isAmp;

        if(!mCollectingButton && !mCollectingHighButton && !mKeepArmDownButton && !mPrepareToShootButton && mAmpButton){
            mArm.setDesiredState(ArmState.AMP);
        } else if(!mCollectingButton && !mCollectingHighButton && !mKeepArmDownButton && !mPrepareToShootButton && !mAmpButton){
            mArm.setDesiredState(ArmState.SAFE);
        }
    }

    public void setArmState(ArmState desiredState) {
        mArm.setDesiredState(desiredState);
    }

    public void zeroNavX() {
        mDriveSystem.zeroNavX();
    }


    public void aimDrivesystem(boolean isAiming) {
        mDriveSystem.aimSpeaker(isAiming);
    }

    public double calculateShortestRotationToAngle(double startingAngle, double goalAngle) {
        double ogAngle = startingAngle - goalAngle;
        double[] angles = { ogAngle, ogAngle - 360, ogAngle + 360 };
        int smallestAngleIndex = -1;

        for (int i = 0; i < angles.length; i++) {
            if (smallestAngleIndex == -1 || Math.abs(angles[i]) < Math.abs(angles[smallestAngleIndex])) {
                smallestAngleIndex = i;
            }
        }

        return angles[smallestAngleIndex];
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
            mDriveSystem.setPosition(position);
            mDriveSystem.setDesiredState(DriveSystemState.DRIVE_SETPOINT);
        } else if (path != null) {
            mDriveSystem.setTrajectoryFromPath(path);
            mDriveSystem.setDesiredState(DriveSystemState.DRIVE_TRAJECTORY);
        } else if (desiredAction != Action.SYSID) {
            mDriveSystem.setDesiredState(DriveSystemState.DRIVE_MANUAL);
        }

        System.out.println(mDriveSystem.getName() + " :: " + mDriveSystem.getState().toString() + " || "
                + mScollector.getName() + " :: " + mScollector.getState().toString() + " || " + mClimber.getName()
                + " :: " + mClimber.getState().toString());
    }

    public boolean stepIsFinished() {
        return !isRunningAction() && mDriveSystem.matchesDesiredState();
    }

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
                mArm.setDesiredState(ArmState.SAFE);
                mDriveSystem.setDesiredState(DriveSystem.DriveSystemState.DRIVE_MANUAL);
                if(getLimelightPose().getX() != -99.9) {
                    mDriveSystem.setOdometry(getLimelightPose());
                }
                break;
            case AUTONOMOUS:
                mDriveSystem.setOdometry(getLimelightPose());
                break;
            default:
                break;
        }
    }

    public void callPrintModules() {
        mDriveSystem.printModules();
    }

    public void run(DriverInput driverInput) {
        mArm.updateRobotPose(mDriveSystem.getRobotPose());

        mDriveSystem.updateVisionLocalization(getLimelightPose());
        mArm.setSpeakerDistance(mDriveSystem.distanceToSpeaker());
        mScollector.setArmReadyToShoot(mArm.matchesDesiredState());

        boolean seesApriltag = mBackLimelightTable.getEntry("tv").getDouble(-1) >= 1;
        final double speedTolerance = 0.1;
        ChassisSpeeds robotSpeeds = mDriveSystem.getChassisSpeeds();
        boolean driveSystemSlowEnough = robotSpeeds.vxMetersPerSecond <= speedTolerance && robotSpeeds.vyMetersPerSecond <= speedTolerance; 
        mSeesAprilTagEntry.setBoolean(seesApriltag);
        if(seesApriltag && driveSystemSlowEnough){
            mDriveSystem.setOdometry(getLimelightPose());
        }


        switch (mCurrentOperatingMode) {
            case TELEOP:
                EVector driverTriggers = driverInput.getTriggerAxis(ConfigMap.DRIVER_CONTROLLER_PORT);
                driverDriving(
                        driverInput.getControllerJoyAxis(ControllerSide.LEFT, ConfigMap.DRIVER_CONTROLLER_PORT),
                        driverInput.getControllerJoyAxis(ControllerSide.RIGHT, ConfigMap.DRIVER_CONTROLLER_PORT),
                        driverTriggers);
                mClimber.manualClimb(
                    driverInput.getControllerJoyAxis(ControllerSide.LEFT, ConfigMap.CO_PILOT_PORT).y
                );
                break;
            case AUTONOMOUS:
                if (mScollector.getState() == ScollectorState.COLLECT
                        || mScollector.getState() == ScollectorState.COLLECT_RAMP
                        || mScollector.getState() == ScollectorState.COLLECT_AUTO_SHOOT) {
                    if (mScollector.hasNote()) {
                        mArm.setDesiredState(ArmState.SUBWOOFER);
                    } else {
                        mArm.setDesiredState(ArmState.COLLECT);
                    }
                }
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

    private Pose2d getLimelightPose(){
        double[] doubleArray = {-99.9, -99.9, -99.9, -99.9, -99.9, -99.9};
        
        doubleArray = mBackLimelightTable.getEntry("botpose").getDoubleArray(doubleArray);
        return new Pose2d(doubleArray[0], doubleArray[1], mDriveSystem.getRobotAngle());
    }

    private void initActions() {
        defineAction(Action.SYSID,
                new SubsystemSetting(mDriveSystem, DriveSystemState.SYSID),
                new SubsystemSetting(mArm, ArmState.COLLECT),
                new SubsystemSetting(mScollector, ScollectorState.IDLE));

        defineAction(Action.COLLECT,
                new SubsystemSetting(mScollector, ScollectorState.COLLECT),
                new SubsystemSetting(mArm, ArmState.COLLECT));

        defineAction(Action.SHOOT,
                new SubsystemSetting(mScollector, ScollectorState.SHOOT),
                new SubsystemSetting(mArm, ArmState.SUBWOOFER));

        defineAction(Action.COLLECT_RAMP,
                new SubsystemSetting(mScollector, ScollectorState.COLLECT_RAMP),
                new SubsystemSetting(mArm, ArmState.COLLECT));

        defineAction(Action.COLLECT_AUTO_SHOOT,
                new SubsystemSetting(mArm, ArmState.AUTO_ANGLE),
                new SubsystemSetting(mScollector, ScollectorState.COLLECT_AUTO_SHOOT));
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

    private void odometryUpdate(double velocityX, double velocityY) {
        // if (velocityX <= ConfigMap.MAX_VELOCITY_FOR_UPDATE || velocityY <=
        // ConfigMap.MAX_VELOCITY_FOR_UPDATE) {
        // if
        // (LimelightHelper.getLatestResults(ConfigMap.LIMELIGHT_NAME).targetingResults.targets_Fiducials.length
        // >= 2) {
        // localizeOnLimelight();
        // }
        // }
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
