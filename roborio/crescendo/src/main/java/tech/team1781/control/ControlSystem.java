package tech.team1781.control;

import java.sql.Time;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Stack;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import tech.team1781.subsystems.*;
import tech.team1781.subsystems.Arm.ArmState;
import tech.team1781.subsystems.DriveSystem.DriveSystemState;
import tech.team1781.subsystems.Scollector.ScollectorState;
import tech.team1781.subsystems.Subsystem.OperatingMode;
import tech.team1781.subsystems.Subsystem.SubsystemState;
import tech.team1781.subsystems.LEDs.LedState;
import tech.team1781.utils.EVector;
import tech.team1781.DriverInput;
import tech.team1781.ShuffleboardStyle;
import tech.team1781.utils.Limelight;
// import tech.team1781.subsystems.Climber.TrapState;
import tech.team1781.utils.NetworkLogger;

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
    private LEDs mLEDs;

    private OperatingMode mCurrentOperatingMode;

    // Slew Rate Limiters for controls
    private final SlewRateLimiter mXDriveLimiter = new SlewRateLimiter(ConfigMap.DRIVER_TRANSLATION_RATE_LIMIT);
    private final SlewRateLimiter mYDriveLimiter = new SlewRateLimiter(ConfigMap.DRIVER_TRANSLATION_RATE_LIMIT);
    private final SlewRateLimiter mRotDriveLimiter = new SlewRateLimiter(ConfigMap.DRIVER_ROTATION_RATE_LIMIT);
    private final ProfiledPIDController mLimelightAimController = new ProfiledPIDController(0.070, 0, 0,
            new TrapezoidProfile.Constraints(1, 0.5));
    private final ProfiledPIDController mNoteAimController = new ProfiledPIDController(0.035, 0, 0,
            new TrapezoidProfile.Constraints(1, 0.5));
    private final ProfiledPIDController mAmpAimController = new ProfiledPIDController(0.035, 0, 0,
            new TrapezoidProfile.Constraints(1, 0.5));
    private final ProfiledPIDController mOdometryController = new ProfiledPIDController(4.1, 0, 0,
            new TrapezoidProfile.Constraints(1, 0.5));

    private boolean mAutoAiming = false;
    private double mAimingAngle = 0.0;
    private double mStrafeDC = 0.0;

    // CURRENT STATE OF INPUT for HOLD DOWN BUTTONS
    // private boolean mKeepArmDownButton = false;
    // private boolean mCollectingButton = false;
    // private boolean mPrepareToShootButton = false;
    // private boolean mShootButton = false;
    // private boolean mSpitButton = false;
    // private boolean mClimberRetractButton = false;
    // private boolean mClimberExtendButton = false;
    private boolean mCenterOnAprilTagButton = false;
    private boolean mAutoCenterAmp = false;
    // private boolean mSeekSpeakerButton = false;
    // private boolean mCollectingHighButton = false;
    // private boolean mAmpButton = false;
    // private boolean mShootPodiumButton = false;
    // private boolean mSkipNoteButton = false;
    // private boolean mLobButton = false;

    private Stack<SubsystemSetting> mSettingStack = new Stack<>();

    private GenericEntry mSeesAprilTagEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Sees AprilTag",
            false, ShuffleboardStyle.SEES_APRILTAG);
    private HashMap<Number, Pose2d> aprilTagCoords = new HashMap<>();

    private Action mCurrentAction = null;
    private SeekNoteState mCurrentSeekNoteState = SeekNoteState.SEEKING;
    private EVector mSeekNoteTargetPose = EVector.newVector(-1, -1, -1);
    private Timer mSeekTimer = new Timer();

    public enum Action {
        COLLECT,
        SHOOT,
        COLLECT_RAMP,
        COLLECT_AUTO_SHOOT,
        SYSID,
        SEEK_NOTE,
        AUTO_AIM_SHOOT,
        SHOOT_FAR,
        OFF_KICKSTAND,
        SHOOT_NOTE_ONE,
        SHOOT_NOTE_TWO,
        SHOOT_NOTE_THREE,
        RAMP_SHOOTER,
        SHOOT_SUBWOOFER,
        SHOOT_SUBWOOFER_NO_AIM,
        REJECT_NOTE,
        COLLECT_RAMP_STAY_DOWN
    }

    public ControlSystem() {
        mDriveSystem = new DriveSystem();
        mScollector = new Scollector();
        mClimber = new Climber();
        mArm = new Arm();
        mLEDs = new LEDs();

        mSubsystems = new ArrayList<>();
        mSubsystems.add(mDriveSystem);
        mSubsystems.add(mScollector);
        mSubsystems.add(mClimber);
        mSubsystems.add(mArm);
        mSubsystems.add(mLEDs);

        aprilTagCoords.put(1, new Pose2d(15.078597, 0.245597, new Rotation2d()));
        aprilTagCoords.put(2, new Pose2d(16.184259, 0.883391, new Rotation2d()));
        aprilTagCoords.put(3, new Pose2d(16.578467, 4.982443, new Rotation2d()));
        aprilTagCoords.put(4, new Pose2d(16.578467, 5.547593, new Rotation2d()));
        aprilTagCoords.put(5, new Pose2d(14.699883, 8.203925, new Rotation2d()));
        aprilTagCoords.put(6, new Pose2d(1.840625, 8.203925, new Rotation2d()));
        aprilTagCoords.put(7, new Pose2d(-0.038975, 5.547593, new Rotation2d()));
        aprilTagCoords.put(8, new Pose2d(-0.038975, 4.982443, new Rotation2d()));
        aprilTagCoords.put(9, new Pose2d(0.355233, 0.883391, new Rotation2d()));
        aprilTagCoords.put(10, new Pose2d(1.460641, 0.245597, new Rotation2d()));
        aprilTagCoords.put(11, new Pose2d(11.903851, 3.712951, new Rotation2d()));
        aprilTagCoords.put(12, new Pose2d(11.903851, 4.498065, new Rotation2d()));
        aprilTagCoords.put(13, new Pose2d(11.219321, 4.104873, new Rotation2d()));
        aprilTagCoords.put(14, new Pose2d(5.319917, 4.104873, new Rotation2d()));
        aprilTagCoords.put(15, new Pose2d(4.640467, 4.498065, new Rotation2d()));
        aprilTagCoords.put(16, new Pose2d(4.640467, 3.712951, new Rotation2d()));

        initActions();

        mStepTime = new Timer();

        NetworkLogger.logData("Current ControlSystem Action", "None");
    }

    public static boolean isRed() {
        return DriverStation.getAlliance().get() == Alliance.Red;
    }

    public void driverDriving(EVector translation, EVector rotation, EVector triggers) {
        boolean isRed = DriverStation.getAlliance().get() == Alliance.Red;
        int mult = isRed ? -1 : 1;
        final double triggermult = 0;
        triggers.mult(triggermult);

        // forward and backwards
        double xVelocity = -translation.y * mult;
        // left and right
        double yVelocity = -translation.x * mult;
        // rotation
        double rotVelocity = -rotation.x * ConfigMap.DRIVER_ROTATION_INPUT_MULTIPIER + ((triggers.x) - (triggers.y));

        mDriveSystem.driveRaw(
                mAutoCenterAmp ? mStrafeDC
                        : mXDriveLimiter.calculate(xVelocity) * ConfigMap.MAX_VELOCITY_METERS_PER_SECOND,
                mYDriveLimiter.calculate(yVelocity) * ConfigMap.MAX_VELOCITY_METERS_PER_SECOND,
                mAutoAiming ? mAimingAngle
                        : (mRotDriveLimiter.calculate(rotVelocity) * ConfigMap.MAX_VELOCITY_RADIANS_PER_SECOND));
    }

    public void keepArmDown(boolean pushingKeepDown) {
        if (pushingKeepDown) {
            mSettingStack.add(new SubsystemSetting(mArm, ArmState.COLLECT));
        }
    }

    public void setCollecting(boolean pushingCollect) {
        if (pushingCollect) {
            mSettingStack.add(new SubsystemSetting(mArm, ArmState.COLLECT));
            mSettingStack.add(new SubsystemSetting(mScollector, ScollectorState.COLLECT));
        }
    }

    public void setSpit(boolean pushingSpit) {
        if (pushingSpit) {
            mSettingStack.add(new SubsystemSetting(mScollector, ScollectorState.SPIT));
        }
    }

    public void setShooting(boolean pushingShoot) {
        if (pushingShoot) {
            mSettingStack.add(new SubsystemSetting(mScollector, ScollectorState.SHOOT));
        }
    }

    public void manualAdjustAngle(double diff) {
        mSettingStack.add(new SubsystemSetting(mArm, ArmState.MANUAL));
        mArm.manualAdjustAngle(diff);
    }

    public void lobNote(boolean isLobbing) {
        if (isLobbing) {
            mSettingStack.add(new SubsystemSetting(mArm, ArmState.LOB));
            mSettingStack.add(new SubsystemSetting(mScollector, ScollectorState.LOB));
        }
    }

    public void calibratePosition() {
        mDriveSystem.setOdometry(new Pose2d(Limelight.getBotPose2d(ConfigMap.APRILTAG_LIMELIGHT).getTranslation(),
                mDriveSystem.getRobotAngle()));
    }

    public void setPrepareToShoot(boolean pushingPrepare) {
        if (pushingPrepare) {
            mSettingStack.add(new SubsystemSetting(mArm, ArmState.AUTO_ANGLE));
            mSettingStack.add(new SubsystemSetting(mScollector, ScollectorState.RAMP_SHOOTER));
        }
    }

    public void setCollectHigh(boolean collectingHigh) {
        if (collectingHigh) {
            mSettingStack.add(new SubsystemSetting(mArm, ArmState.COLLECT_HIGH));
            mSettingStack.add(new SubsystemSetting(mScollector, ScollectorState.COLLECT));
        }
    }

    public void setAmp(boolean isAmp) {
        if (isAmp) {
            mSettingStack.add(new SubsystemSetting(mArm, ArmState.AMP));
        }
    }

    public void setArmState(ArmState desiredState) {
        mArm.setDesiredState(desiredState);
    }

    public void zeroNavX() {
        mDriveSystem.zeroNavX();
    }

    public void setCenteringOnAprilTag(boolean isHeld) {
        if (isHeld != mCenterOnAprilTagButton) {
            mLimelightAimController.reset(0);

            if (!isHeld) {
                mAimingAngle = 0;
            }
        }
        mCenterOnAprilTagButton = isHeld;
    }

    public void setCenteringOnAmp(boolean isHeld) {
        mAutoCenterAmp = isHeld;
    }

    public void autoAimingInputs() {
        mSeesAprilTagEntry.setBoolean(Limelight.getTX(ConfigMap.APRILTAG_LIMELIGHT) != 0.0);

        if (!mCenterOnAprilTagButton) {
            mAutoAiming = false;
        }

        if (mCenterOnAprilTagButton && !mAutoCenterAmp) {
            if (isRed()) {
                centerOnAprilTag(ConfigMap.RED_SPEAKER_APRILTAG);
            } else {
                centerOnAprilTag(ConfigMap.BLUE_SPEAKER_APRILTAG);
            }
            mAutoAiming = true;
        }

        if (mAutoCenterAmp) {
            strafeToAprilTag();
        } else {
            mStrafeDC = 0;
        }

    }

    public void centerOnAprilTag(int id) {
        Limelight.setTargetApriltag(ConfigMap.APRILTAG_LIMELIGHT,
                isRed() ? ConfigMap.RED_SPEAKER_APRILTAG : ConfigMap.BLUE_SPEAKER_APRILTAG);
        double x = Limelight.getTX(ConfigMap.APRILTAG_LIMELIGHT);

        if (x != 0.0) {
            mAimingAngle = mLimelightAimController.calculate(x, 0);
        } else {
            odometryAlignment(id);
            // mAimingAngle = 0.0;
        }
    }

    public void strafeToAprilTag() {
        double tx = Limelight.getTX(ConfigMap.NOTE_LIMELIGHT);

        if (tx == 0) {
            mStrafeDC = 0;
            return;
        }

        mStrafeDC = -mAmpAimController.calculate(tx, 0);
    }

    public void odometryAlignment(int id) {
        EVector robotPose = EVector.fromPose(mDriveSystem.getRobotPose());
        EVector targetPose = EVector.fromPose(aprilTagCoords.get(id));

        robotPose.z = 0;

        double angle = robotPose.angleBetween(targetPose) - Math.PI;
        angle = normalizeRadians(angle);

        mAimingAngle = mLimelightAimController.calculate(mDriveSystem.getRobotAngle().getRadians(), angle);
    }

    private double normalizeRadians(double rads) {
        rads %= 2 * Math.PI;

        if (rads < 0) {
            rads += 2 * Math.PI;
        }

        return rads;
    }

    public void shootPodium(boolean isShooting) {
        // if (isShooting == mShootPodiumButton) {
        // return;
        // }

        // mShootPodiumButton = isShooting;

        // if (mShootPodiumButton && !mCollectingButton && !mPrepareToShootButton) {
        // if (!mKeepArmDownButton)
        // mArm.setDesiredState(ArmState.PODIUM);
        // mScollector.setDesiredState(ScollectorState.RAMP_SHOOTER);

        // } else if (!mCollectingButton && !mPrepareToShootButton) {
        // if (!mKeepArmDownButton)
        // mArm.setDesiredState(ArmState.SAFE);
        // mScollector.setDesiredState(ScollectorState.IDLE);
        // }
        if (isShooting) {
            mSettingStack.add(new SubsystemSetting(mArm, ArmState.PODIUM));
            mSettingStack.add(new SubsystemSetting(mScollector, ScollectorState.RAMP_SHOOTER));
        }
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

    public void centerNote() {
        double x = Limelight.getTX(ConfigMap.NOTE_LIMELIGHT);
        if (x != 0.0) {
            mAimingAngle = mNoteAimController.calculate(x, 0);
        } else {
            mAimingAngle = 0.0;
        }
    }

    // public void setAction(Action desiredAction) {
    // setAutoStep(desiredAction, null, null);
    // }

    // public void setAutoStep(Action desiredAction, EVector position,
    // PathPlannerPath path) {
    // mStepTime.reset();
    // mStepTime.start();

    // mSeekTimer.reset();
    // mSeekTimer.start();

    // if (desiredAction != null) {
    // mCurrentSettings = mActions.get(desiredAction);
    // try {
    // for (SubsystemSetting setting : mCurrentSettings) {
    // setting.setState();
    // }
    // } catch (NullPointerException e) {
    // }

    // mCurrentAction = desiredAction;

    // if (mCurrentAction == Action.SEEK_NOTE) {
    // mCurrentSeekNoteState = SeekNoteState.SEEKING;
    // mSeekNoteTargetPose = EVector.newVector(-1, -1, -1);
    // }
    // } else {
    // mCurrentSettings = null;
    // }

    // if (position != null) {
    // mDriveSystem.setPosition(position);
    // mDriveSystem.setDesiredState(DriveSystemState.DRIVE_SETPOINT);
    // } else if (path != null) {
    // mDriveSystem.setTrajectoryFromPath(path);
    // mDriveSystem.setDesiredState(DriveSystemState.DRIVE_TRAJECTORY);
    // } else if (desiredAction != Action.SYSID && desiredAction !=
    // Action.SEEK_NOTE) {
    // mDriveSystem.setDesiredState(DriveSystemState.DRIVE_MANUAL);
    // }

    // System.out.println(mDriveSystem.getName() + " :: " +
    // mDriveSystem.getState().toString() + " || "
    // + mScollector.getName() + " :: " + mScollector.getState().toString() + " || "
    // + mClimber.getName()
    // + " :: " + mClimber.getState().toString());
    // }

    public void setAutoStep(AutoStep step) {
        if (step == null) {
            return;
        }

        mCurrentSettings = null;
        mCurrentAction = null;
        mStepTime.reset();
        mStepTime.start();

        switch (step.getType()) {
            case ACTION:
                setAction(step.getAction());
                mDriveSystem.setDesiredState(DriveSystemState.DRIVE_MANUAL);
                break;
            case PATH:
                mDriveSystem.setTrajectoryFromPath(step.getPath());
                mDriveSystem.setDesiredState(DriveSystemState.DRIVE_TRAJECTORY);
                break;
            case POSITION:
                mDriveSystem.setWaypoint(step.getWaypointHolder());
                mDriveSystem.setDesiredState(DriveSystemState.DRIVE_SETPOINT);
                break;
            case PATH_AND_ACTION:
                mDriveSystem.setTrajectoryFromPath(step.getPath());
                mDriveSystem.setDesiredState(DriveSystemState.DRIVE_TRAJECTORY);
                setAction(step.getAction());
                break;
            case POSITION_AND_ACTION:
                mDriveSystem.setWaypoint(step.getWaypointHolder());
                mDriveSystem.setDesiredState(DriveSystemState.DRIVE_SETPOINT);
                setAction(step.getAction());
                break;
            case NOTE_POSITION:
                mDriveSystem.setWaypoint(step.getWaypointHolder());
                mDriveSystem.setDesiredState(DriveSystemState.DRIVE_SETPOINT);
                mDriveSystem.setDesiredState(DriveSystemState.DRIVE_NOTE);
                setAction(step.getAction());
                break;
            case NOTE_TRAJECTORY:
                mDriveSystem.setTrajectoryFromPath(step.getPath());
                mDriveSystem.setDesiredState(DriveSystemState.DRIVE_TRAJECTORY_NOTE);
                setAction(step.getAction());
                break;
            case ROTATION:
                mDriveSystem.setRotation(step.getWaypointHolder().getPosition().z);
                mDriveSystem.setDesiredState(DriveSystemState.DRIVE_ROTATION);
                break;
            case ROTATION_AND_ACTION:
                mDriveSystem.setRotation(step.getWaypointHolder().getPosition().z);
                mDriveSystem.setDesiredState(DriveSystemState.DRIVE_ROTATION);
                setAction(step.getAction());
                break;
            case WAIT:
                mDriveSystem.setDesiredState(DriveSystemState.DRIVE_MANUAL);
                break;
        }
    }

    public void setAction(Action desiredAction) {
        mSeekTimer.reset();
        mSeekTimer.start();

        mCurrentSettings = mActions.get(desiredAction);
        try {
            for (SubsystemSetting setting : mCurrentSettings) {
                setting.setState();
            }
        } catch (NullPointerException e) {
        }

        mCurrentAction = desiredAction;

        if (mCurrentAction == Action.SEEK_NOTE) {
            mCurrentSeekNoteState = SeekNoteState.SEEKING;
            mSeekNoteTargetPose = EVector.newVector(-1, -1, -1);
        } else if (mCurrentAction == Action.AUTO_AIM_SHOOT) {
            mDriveSystem.setDesiredState(DriveSystemState.DRIVE_MANUAL);
        }

    }

    public boolean stepIsFinished() {
        if (mCurrentAction == null && mDriveSystem.getState() == DriveSystemState.DRIVE_MANUAL) {
            return false;
        }
        if (mCurrentAction == Action.SEEK_NOTE) {
            return mCurrentSeekNoteState == SeekNoteState.DONE;
        }

        if (mDriveSystem.getState() == DriveSystemState.DRIVE_NOTE) {
            return mScollector.hasNote() || mScollector.noteCloseToShooter();
        }

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
                Limelight.setPipeline(ConfigMap.NOTE_LIMELIGHT, ConfigMap.NOTE_LIMELIGHT_APRILTAG_PIPELINE);

                mSettingStack.clear();
                mXDriveLimiter.reset(0);
                mYDriveLimiter.reset(0);
                mRotDriveLimiter.reset(0);
                mDriveSystem.setDesiredState(DriveSystem.DriveSystemState.DRIVE_MANUAL);
                if (Limelight.getTX(ConfigMap.APRILTAG_LIMELIGHT) != 0.0) {
                    mDriveSystem.setOdometry(Limelight.getBotPose2d(ConfigMap.APRILTAG_LIMELIGHT));
                }

                break;
            case AUTONOMOUS:
                System.out.println();
                Limelight.setPipeline(ConfigMap.NOTE_LIMELIGHT, ConfigMap.NOTE_LIMELIGHT_NOTE_PIPELINE);
                break;
            default:
                break;
        }
    }

    public void run(DriverInput driverInput) {
        NetworkLogger.logData("Current ControlSystem Action", mCurrentAction);

        mArm.updateAimSpots(mDriveSystem.getRobotPose());

        mScollector.setArmReadyToShoot(mArm.matchesDesiredState());
        if (mDriveSystem.badRoll()) {
            mLEDs.setDesiredState(LedState.BAD_ROLL);
        } else if (mScollector.hasNote()) {
            mLEDs.setDesiredState(LedState.HAS_NOTE);
        } else {
            mLEDs.setDesiredState(LedState.NO_NOTE);
        }

        switch (mCurrentOperatingMode) {
            case TELEOP:
                SubsystemState finalScollectorState = ScollectorState.IDLE;
                SubsystemState finalArmState = mArm.getState() == ArmState.MANUAL ? ArmState.MANUAL : ArmState.SAFE;
                SubsystemState finalDriveState = DriveSystemState.DRIVE_MANUAL;

                while (!mSettingStack.isEmpty()) {
                    SubsystemSetting setting = mSettingStack.pop();
                    Subsystem subsystem = setting.getSubsystem();
                    SubsystemState state = setting.getState();

                    if (subsystem == mDriveSystem) {
                        finalDriveState = state;
                    } else if (subsystem == mScollector) {
                        if (state == ScollectorState.COLLECT) {
                            if (finalScollectorState == ScollectorState.RAMP_SHOOTER) {
                                finalScollectorState = ScollectorState.COLLECT_RAMP;
                                continue;
                            }
                            if (finalScollectorState == ScollectorState.LOB) {
                                finalScollectorState = ScollectorState.COLLECT_RAMP_LOB;
                                continue;
                            }
                        }
                        finalScollectorState = state;
                    } else if (subsystem == mArm) {
                        if (mArm.getState() == ArmState.MANUAL && state == ArmState.AUTO_ANGLE) {
                            continue;
                        }
                        finalArmState = state;
                    }
                }

                mDriveSystem.setDesiredState(finalDriveState);
                mScollector.setDesiredState(finalScollectorState);
                mArm.setDesiredState(finalArmState);

                localizationUpdates();
                EVector driverTriggers = driverInput.getTriggerAxis(ConfigMap.DRIVER_CONTROLLER_PORT);
                driverDriving(
                        driverInput.getControllerJoyAxis(ControllerSide.LEFT, ConfigMap.DRIVER_CONTROLLER_PORT),
                        driverInput.getControllerJoyAxis(ControllerSide.RIGHT, ConfigMap.DRIVER_CONTROLLER_PORT),
                        driverTriggers);
                if (mArm.getState() == ArmState.COLLECT) {
                    mClimber.manualClimb(
                            -1,
                            false);
                } else {
                    mClimber.manualClimb(
                            -driverInput.getControllerJoyAxis(ControllerSide.LEFT, ConfigMap.CO_PILOT_PORT).y,
                            driverInput.getTriggerAxis(ConfigMap.CO_PILOT_PORT).x < 0.5);
                }
                autoAimingInputs();
                break;
            case AUTONOMOUS:
                localizationUpdates();
                if (mScollector.getState() == ScollectorState.COLLECT
                        || (mScollector.getState() == ScollectorState.COLLECT_RAMP
                                && (mDriveSystem.getState() == DriveSystemState.DRIVE_NOTE
                                        || mDriveSystem.getState() == DriveSystemState.DRIVE_SETPOINT || mDriveSystem.getState() == DriveSystemState.DRIVE_TRAJECTORY))
                        || (mScollector.getState() == ScollectorState.COLLECT_AUTO_SHOOT
                                && (mArm.getState() != ArmState.NOTE_ONE && mArm.getState() != ArmState.NOTE_THREE))) {
                    if (mScollector.hasNote() || mScollector.noteCloseToShooter()) {
                        mArm.setDesiredState(ArmState.AUTO_ANGLE);
                    } else {
                        mArm.setDesiredState(ArmState.COLLECT);
                    }
                }

                if ((mCurrentAction == Action.AUTO_AIM_SHOOT || mCurrentAction == Action.SHOOT_NOTE_ONE
                        || mCurrentAction == Action.SHOOT_NOTE_TWO
                        || mCurrentAction == Action.SHOOT_NOTE_THREE || mCurrentAction == Action.SHOOT_SUBWOOFER)
                        && mDriveSystem.getState() == DriveSystem.DriveSystemState.DRIVE_MANUAL) {
                    centerOnAprilTag(isRed() ? ConfigMap.RED_SPEAKER_APRILTAG : ConfigMap.BLUE_SPEAKER_APRILTAG);
                    mDriveSystem.driveRaw(0, 0, mAimingAngle);
                } else if (mDriveSystem.getState() == DriveSystemState.DRIVE_MANUAL
                        && mCurrentAction != Action.SEEK_NOTE) {
                    mDriveSystem.driveRaw(0, 0, 0);
                    mAimingAngle = 0.0;
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

    public void disabledPeriodic() {
        for (Subsystem subsystem : mSubsystems) {
            subsystem.disabledPeriodic();
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
        System.out.println("&&&&&&&&&& INTERRUPTING ACTION &&&&&&&&&");
        mCurrentSettings = null;
        mCurrentAction = null;

        for (Subsystem s : mSubsystems) {
            s.restoreDefault();
        }
    }

    public void localizationUpdates() {
        final double speedTolerance = 0.1;
        final double DIST_TOLERANCE = 0.5;
        ChassisSpeeds robotSpeeds = mDriveSystem.getChassisSpeeds();
        EVector chassisSpeedsVector = new EVector(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
        boolean driveSystemSlowEnough = chassisSpeedsVector.magnitude() <= speedTolerance;
        Pose2d limelightPoseTemp = Limelight.getBotPose2d(ConfigMap.APRILTAG_LIMELIGHT);
        Pose2d limelightPose = new Pose2d(limelightPoseTemp.getTranslation(), mDriveSystem.getRobotAngle());

        double dist = EVector.fromPose(limelightPose).dist(EVector.fromPose(mDriveSystem.getRobotPose()));
        // if (driveSystemSlowEnough && limelightPose.getY() != 0.0 && limelightPose.getX() != 0.0
        //         && Limelight.getNumberOfApriltags(ConfigMap.APRILTAG_LIMELIGHT) > 1) {
        //     if (dist > DIST_TOLERANCE) {
        //         mDriveSystem.setOdometry(limelightPose);
        //     }
        // } else 
       // if (limelightPose.getY() != 0.0 && limelightPose.getX() != 0.0 && dist >= DIST_TOLERANCE) {
            mDriveSystem.updateVisionLocalization(limelightPose);
       // }
    }


    

    private void initActions() {
        defineAction(Action.SHOOT_SUBWOOFER_NO_AIM,
                new SubsystemSetting(mScollector, ScollectorState.SHOOT_ASAP),
                new SubsystemSetting(mArm, ArmState.SUBWOOFER));

        defineAction(Action.SHOOT_FAR,
                new SubsystemSetting(mScollector, ScollectorState.SHOOT_ASAP),
                new SubsystemSetting(mArm, ArmState.FAR_SHOT));

        defineAction(Action.REJECT_NOTE,
                new SubsystemSetting(mArm, ArmState.COLLECT),
                new SubsystemSetting(mScollector, ScollectorState.SPIT));

        defineAction(Action.SHOOT_NOTE_TWO,
                new SubsystemSetting(mArm, ArmState.NOTE_TWO),
                new SubsystemSetting(mScollector, ScollectorState.COLLECT_AUTO_SHOOT));

        defineAction(Action.RAMP_SHOOTER,
                new SubsystemSetting(mArm, ArmState.SAFE),
                new SubsystemSetting(mScollector, ScollectorState.RAMP_SHOOTER));

        defineAction(Action.SHOOT_SUBWOOFER,
                new SubsystemSetting(mArm, ArmState.SUBWOOFER),
                new SubsystemSetting(mScollector, ScollectorState.SHOOT_ASAP));

        defineAction(Action.SYSID,
                new SubsystemSetting(mDriveSystem, DriveSystemState.SYSID),
                new SubsystemSetting(mArm, ArmState.COLLECT),
                new SubsystemSetting(mScollector, ScollectorState.IDLE));

        defineAction(Action.AUTO_AIM_SHOOT,
                new SubsystemSetting(mDriveSystem, DriveSystemState.DRIVE_MANUAL),
                new SubsystemSetting(mArm, ArmState.AUTO_ANGLE),
                new SubsystemSetting(mScollector, ScollectorState.COLLECT_AUTO_SHOOT));

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

        defineAction(Action.OFF_KICKSTAND,
                new SubsystemSetting(mArm, ArmState.KICKSTAND),
                new SubsystemSetting(mScollector, ScollectorState.RAMP_SHOOTER));

        defineAction(Action.SHOOT_NOTE_ONE,
                new SubsystemSetting(mArm, ArmState.NOTE_ONE),
                new SubsystemSetting(mScollector, ScollectorState.COLLECT_AUTO_SHOOT));

        defineAction(Action.SHOOT_NOTE_THREE,
                new SubsystemSetting(mArm, ArmState.NOTE_THREE),
                new SubsystemSetting(mScollector, ScollectorState.COLLECT_AUTO_SHOOT));

        defineAction(Action.COLLECT_RAMP_STAY_DOWN,
                new SubsystemSetting(mArm, ArmState.COLLECT),
                new SubsystemSetting(mScollector, ScollectorState.COLLECT_RAMP));

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

    public void disabledLighting() {
        mLEDs.setDesiredState(LedState.DEFAULT);
    }

    private enum SeekNoteState {
        SEEKING,
        COLLECTING,
        DONE
    }

}

class SubsystemSetting {
    public Subsystem mSubsystem;
    public SubsystemState mState;

    public SubsystemSetting(Subsystem subsystem, SubsystemState state) {
        mSubsystem = subsystem;
        mState = state;
    }

    public void setState() {
        mSubsystem.setDesiredState(mState);
    }

    public SubsystemState getState() {
        return mState;
    }

    public Subsystem getSubsystem() {
        return mSubsystem;
    }

    public boolean isFinished() {
        boolean ret_val = mSubsystem.matchesDesiredState();

        return ret_val;
    }

}
