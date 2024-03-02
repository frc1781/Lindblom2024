package tech.team1781.control;

import java.util.ArrayList;
import java.util.HashMap;

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
import tech.team1781.utils.Limelight;

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
    private final ProfiledPIDController mLimelightAimController = new ProfiledPIDController(0.070, 0, 0,
            new TrapezoidProfile.Constraints(1, 0.5));
    private final ProfiledPIDController mNoteAimController = new ProfiledPIDController(0.035, 0, 0,
            new TrapezoidProfile.Constraints(1, 0.5));
    private final ProfiledPIDController mOdometryController = new ProfiledPIDController(4.1, 0, 0,
            new TrapezoidProfile.Constraints(1, 0.5));

    private boolean mAutoAiming = false;
    private double mAimingAngle = 0.0;

    // CURRENT STATE OF INPUT for HOLD DOWN BUTTONS
    private boolean mKeepArmDownButton = false;
    private boolean mCollectingButton = false;
    private boolean mPrepareToShootButton = false;
    private boolean mShootButton = false;
    private boolean mSpitButton = false;
    private boolean mClimberRetractButton = false;
    private boolean mClimberExtendButton = false;
    private boolean mCenterOnAprilTagButton = false;
    private boolean mAutoCollectionButton = false;
    private boolean mSeekSpeakerButton = false;
    private boolean mCollectingHighButton = false;
    private boolean mAmpButton = false;
    private boolean mShootPodiumButton = false;
    private boolean mSkipNoteButton = false;

    private GenericEntry mSeesAprilTagEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Sees AprilTag",
            false, ShuffleboardStyle.SEES_APRILTAG);
    private HashMap<Number, Pose2d> aprilTagCoords = new HashMap<>();

    private Action mCurrentAction = null;
    private SeekNoteState mCurrentSeekNoteState = SeekNoteState.SEEKING;
    private EVector mSeekNoteTargetPose = EVector.newVector(-1, -1, -1);

    public enum Action {
        COLLECT,
        SHOOT,
        COLLECT_RAMP,
        COLLECT_AUTO_SHOOT,
        SYSID,
        SEEK_NOTE,
        AUTO_AIM_SHOOT
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
    }

    public static boolean isRed() {
        return DriverStation.getAlliance().get() == Alliance.Red;
    }

    public void driverDriving(EVector translation, EVector rotation, EVector triggers) {
        boolean isRed = DriverStation.getAlliance().get() == Alliance.Red;
        int mult = isRed ? -1 : 1;
        final double triggermult = 0.1;
        triggers.mult(triggermult);

        // forward and backwards
        double xVelocity = -translation.y * mult;
        // left and right
        double yVelocity = -translation.x * mult;
        // rotation
        double rotVelocity = -rotation.x * 0.25 + ((triggers.x) - (triggers.y));

        mDriveSystem.driveRaw(
                mXDriveLimiter.calculate(xVelocity) * ConfigMap.MAX_VELOCITY_METERS_PER_SECOND,
                mYDriveLimiter.calculate(yVelocity) * ConfigMap.MAX_VELOCITY_METERS_PER_SECOND,
                mAutoAiming ? mAimingAngle
                        : (mRotDriveLimiter.calculate(rotVelocity) * ConfigMap.MAX_VELOCITY_RADIANS_PER_SECOND));

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

    public void skipNote(boolean isSkipping) {
        if (mSkipNoteButton == isSkipping) {
            return;
        }

        mSkipNoteButton = isSkipping;

        if (!mCollectingButton && !mPrepareToShootButton && !mKeepArmDownButton && !mShootPodiumButton) {
            if (!mSkipNoteButton) {
                mArm.setDesiredState(ArmState.SAFE);
                mScollector.setDesiredState(ScollectorState.IDLE);
            } else {
                mArm.setDesiredState(ArmState.SKIP);
                mScollector.setDesiredState(ScollectorState.RAMP_SHOOTER);
            }
        }

    }

    public void calibratePosition() {
        mDriveSystem.setOdometry(Limelight.getBotPose2d(ConfigMap.APRILTAG_LIMELIGHT));
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
        if (mCollectingHighButton == collectingHigh) {
            return;
        }

        mCollectingHighButton = collectingHigh;

        if (!mCollectingButton && collectingHigh && !mKeepArmDownButton) {
            mArm.setDesiredState(ArmState.COLLECT_HIGH);
            mScollector.setDesiredState(ScollectorState.COLLECT);
        }

        if (!collectingHigh && !mKeepArmDownButton && !mPrepareToShootButton && !mCollectingButton) {
            mArm.setDesiredState(ArmState.SAFE);
            mScollector.setDesiredState(ScollectorState.IDLE);
        }
    }

    public void setAmp(boolean isAmp) {
        if (mAmpButton == isAmp) {
            return;
        }

        mAmpButton = isAmp;

        if (!mCollectingButton && !mCollectingHighButton && !mKeepArmDownButton && !mPrepareToShootButton
                && mAmpButton) {
            mArm.setDesiredState(ArmState.AMP);
        } else if (!mCollectingButton && !mCollectingHighButton && !mKeepArmDownButton && !mPrepareToShootButton
                && !mAmpButton) {
            mArm.setDesiredState(ArmState.SAFE);
        }
    }

    public void setArmState(ArmState desiredState) {
        mArm.setDesiredState(desiredState);
    }

    public void zeroNavX() {
        mDriveSystem.zeroNavX();
    }

    public void setCenteringOnAprilTag(boolean isHeld) {
        mCenterOnAprilTagButton = isHeld;
    }

    public void setAutoCollectionButton(boolean isHeld) {
        mAutoCollectionButton = isHeld;
    }

    public void autoAimingInputs() {
        mSeesAprilTagEntry.setBoolean(Limelight.getTX(ConfigMap.APRILTAG_LIMELIGHT) != 0.0);

        if (!mAutoCollectionButton && !mCenterOnAprilTagButton) {
            mAutoAiming = false;
            return;
        }

        if (mCenterOnAprilTagButton && !mAutoCollectionButton) {
            if (isRed()) {
                centerOnAprilTag(ConfigMap.RED_SPEAKER_APRILTAG);
            } else {
                centerOnAprilTag(ConfigMap.BLUE_SPEAKER_APRILTAG);
            }
            mAutoAiming = true;
        }

        if (mAutoCollectionButton && !mCenterOnAprilTagButton) {
            centerNote();
            mAutoAiming = true;
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
        }

    }

    public void odometryAlignment(int id) {
        EVector robotPose = EVector.fromPose(mDriveSystem.getRobotPose());
        EVector targetPose = EVector.fromPose(aprilTagCoords.get(id));

        robotPose.z = 0;

        double angle = robotPose.angleBetween(targetPose) - Math.PI;
        angle = normalizeRadians(angle);

        mAimingAngle = mOdometryController.calculate(mDriveSystem.getRobotAngle().getRadians(), angle);
    }

    private double normalizeRadians(double rads) {
        rads %= 2 * Math.PI;

        if (rads < 0) {
            rads += 2 * Math.PI;
        }

        return rads;
    }

    public void shootPodium(boolean isShooting) {
        if (isShooting == mShootPodiumButton) {
            return;
        }

        mShootPodiumButton = isShooting;

        if (mShootPodiumButton && !mCollectingButton && !mPrepareToShootButton) {
            if (!mKeepArmDownButton)
                mArm.setDesiredState(ArmState.PODIUM);
            mScollector.setDesiredState(ScollectorState.RAMP_SHOOTER);

        } else if (!mCollectingButton && !mPrepareToShootButton) {
            if (!mKeepArmDownButton)
                mArm.setDesiredState(ArmState.SAFE);
            mScollector.setDesiredState(ScollectorState.IDLE);
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
        }
    }

    public void setAction(Action desiredAction) {
        setAutoStep(desiredAction, null, null);
    }

    public void setAutoStep(Action desiredAction, EVector position, PathPlannerPath path) {
        mStepTime.reset();
        mStepTime.start();

        if (desiredAction != null) {
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
        } else if (desiredAction != Action.SYSID && desiredAction != Action.SEEK_NOTE) {
            mDriveSystem.setDesiredState(DriveSystemState.DRIVE_MANUAL);
        }

        System.out.println(mDriveSystem.getName() + " :: " + mDriveSystem.getState().toString() + " || "
                + mScollector.getName() + " :: " + mScollector.getState().toString() + " || " + mClimber.getName()
                + " :: " + mClimber.getState().toString());
    }

    public boolean stepIsFinished() {
        if (mCurrentAction == Action.SEEK_NOTE) {
            return mCurrentSeekNoteState == SeekNoteState.DONE;
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
                mXDriveLimiter.reset(0);
                mYDriveLimiter.reset(0);
                mRotDriveLimiter.reset(0);
                mArm.setDesiredState(ArmState.SAFE);
                mDriveSystem.setDesiredState(DriveSystem.DriveSystemState.DRIVE_MANUAL);
                if (Limelight.getTX(ConfigMap.APRILTAG_LIMELIGHT) != 0.0) {
                    mDriveSystem.setOdometry(Limelight.getBotPose2d(ConfigMap.APRILTAG_LIMELIGHT));
                }

                break;
            case AUTONOMOUS:
                break;
            default:
                break;
        }
    }

    public void run(DriverInput driverInput) {
        mArm.updateAimSpots(mDriveSystem.getRobotPose());

        mScollector.setArmReadyToShoot(mArm.matchesDesiredState());

        localizationUpdates();
        switch (mCurrentOperatingMode) {
            case TELEOP:
                seesNote();
                EVector driverTriggers = driverInput.getTriggerAxis(ConfigMap.DRIVER_CONTROLLER_PORT);
                driverDriving(
                        driverInput.getControllerJoyAxis(ControllerSide.LEFT, ConfigMap.DRIVER_CONTROLLER_PORT),
                        driverInput.getControllerJoyAxis(ControllerSide.RIGHT, ConfigMap.DRIVER_CONTROLLER_PORT),
                        driverTriggers);
                mClimber.manualClimb(
                        driverInput.getControllerJoyAxis(ControllerSide.LEFT, ConfigMap.CO_PILOT_PORT).y);
                autoAimingInputs();
                break;
            case AUTONOMOUS:
                if (mScollector.getState() == ScollectorState.COLLECT
                        || mScollector.getState() == ScollectorState.COLLECT_RAMP
                        || mScollector.getState() == ScollectorState.COLLECT_AUTO_SHOOT) {
                    if (mScollector.hasNote()) {
                        mArm.setDesiredState(ArmState.AUTO_ANGLE);
                    } else {
                        mArm.setDesiredState(ArmState.COLLECT);
                    }
                }

                if (mCurrentAction == Action.SEEK_NOTE) {
                    seekNote();
                }

                if (mCurrentAction == Action.AUTO_AIM_SHOOT
                        && mDriveSystem.getState() == DriveSystem.DriveSystemState.DRIVE_MANUAL) {
                    centerOnAprilTag(isRed() ? ConfigMap.RED_SPEAKER_APRILTAG : ConfigMap.BLUE_SPEAKER_APRILTAG);
                    mDriveSystem.driveRaw(0, 0, mAimingAngle);
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
        System.out.println("&&&&&&&&&& INTERRUPTING ACTION &&&&&&&&&");
        mCurrentSettings = null;
        mCurrentAction = null;

        for (Subsystem s : mSubsystems) {
            s.restoreDefault();
        }
    }

    public void localizationUpdates() {
        final double speedTolerance = 0.1;
        ChassisSpeeds robotSpeeds = mDriveSystem.getChassisSpeeds();
        EVector chassisSpeedsVector = new EVector(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
        boolean driveSystemSlowEnough = chassisSpeedsVector.magnitude() <= speedTolerance;
        Pose2d limelightPoseTemp = Limelight.getBotPose2d(ConfigMap.APRILTAG_LIMELIGHT);
        Pose2d limelightPose = new Pose2d(limelightPoseTemp.getTranslation(), mDriveSystem.getRobotAngle());
        double tx = Limelight.getTX(ConfigMap.APRILTAG_LIMELIGHT);
        if (driveSystemSlowEnough && tx != 0.0 && limelightPose.getY() != 0.0 && limelightPose.getX() != 0.0) {
            double dist = EVector.fromPose(limelightPose).dist(EVector.fromPose(mDriveSystem.getRobotPose()));
            if (dist > 2) {
                mDriveSystem.setOdometry(limelightPose);
            }
        } else if (tx != 0.0 && tx != 0.0 && limelightPose.getY() != 0.0 && limelightPose.getX() != 0.0) {
            mDriveSystem.updateVisionLocalization(limelightPose);
        }
    }

    private boolean seesNote() {
        return Limelight.getTV(ConfigMap.NOTE_LIMELIGHT) == 1;
    }

    private double getNoteDistance() {
        if (!seesNote()) {
            return -1;
        }

        final double COEFFICIENT = 1.69;
        final double EXP = 0.48;
        double area = Limelight.getTA(ConfigMap.NOTE_LIMELIGHT);

        return COEFFICIENT / Math.pow(area, EXP);
    }

    private void initActions() {
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

    private void seekNote() {
        switch (mCurrentSeekNoteState) {
            case SEEKING:
                mDriveSystem.setDesiredState(DriveSystemState.DRIVE_MANUAL);
                mArm.setDesiredState(ArmState.SAFE);
                mScollector.setDesiredState(ScollectorState.IDLE);

                double rotDutyCycle = 0.0;

                if (seesNote()) {
                    centerNote();
                    rotDutyCycle = mAimingAngle;
                    final double TOLERANCE = 0.1;

                    if (Math.abs(rotDutyCycle) <= TOLERANCE && mArm.matchesDesiredState()) {
                        double robotAngle = mDriveSystem.getRobotAngle().getRadians();
                        EVector robotPose = EVector.fromPose(mDriveSystem.getRobotPose());

                        double noteDistance = getNoteDistance();

                        if (noteDistance != -1) {
                            double EXTRA_DISTANCE = 0.25;
                            noteDistance += EXTRA_DISTANCE;
                            EVector notePose = EVector.newVector(
                                    Math.cos(robotAngle) * noteDistance + robotPose.x,
                                    Math.sin(robotAngle) * noteDistance + robotPose.y,
                                    robotAngle);

                            mDriveSystem.setPosition(notePose);
                            mDriveSystem.setDesiredState(DriveSystemState.DRIVE_SETPOINT);
                        }

                        mCurrentSeekNoteState = SeekNoteState.COLLECTING;
                    }
                } else {
                    final double ROT_SPEED = 0.5;
                    rotDutyCycle = ROT_SPEED * (isRed() ? 1 : -1);
                }

                mDriveSystem.driveRaw(0, 0, rotDutyCycle);

                break;
            case COLLECTING:
                mArm.setDesiredState(ArmState.COLLECT);
                mScollector.setDesiredState(ScollectorState.COLLECT);

                if (mScollector.hasNote()) {
                    // mCurrentSeekNoteState = SeekNoteState.DONE;
                }
                break;
            default:
                break;
        }

    }

    private enum SeekNoteState {
        SEEKING,
        COLLECTING,
        DONE
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
