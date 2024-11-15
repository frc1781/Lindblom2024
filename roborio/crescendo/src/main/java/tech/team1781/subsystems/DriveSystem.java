package tech.team1781.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import tech.team1781.ConfigMap;
import tech.team1781.autonomous.AutoStep;
import tech.team1781.autonomous.WaypointHolder;
import tech.team1781.control.ControlSystem;
import tech.team1781.swerve.KrakenL2SwerveModule;
import tech.team1781.swerve.SwerveModule;
import tech.team1781.utils.EEGeometryUtil;
import tech.team1781.utils.EVector;
import tech.team1781.utils.Limelight;

import org.littletonrobotics.junction.Logger;

public class DriveSystem extends Subsystem {

    // Swerve Modules
    private final SwerveModule mFrontLeft = new KrakenL2SwerveModule("Front Left Module",
            ConfigMap.FRONT_LEFT_MODULE_DRIVE_MOTOR,
            ConfigMap.FRONT_LEFT_MODULE_STEER_MOTOR, ConfigMap.FRONT_LEFT_MODULE_STEER_ENCODER,
            ConfigMap.FRONT_LEFT_MODULE_STEER_OFFSET);
    private final SwerveModule mFrontRight = new KrakenL2SwerveModule("Front Right Module",
            ConfigMap.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            ConfigMap.FRONT_RIGHT_MODULE_STEER_MOTOR, ConfigMap.FRONT_RIGHT_MODULE_STEER_ENCODER,
            ConfigMap.FRONT_RIGHT_MODULE_STEER_OFFSET);
    private final SwerveModule mBackLeft = new KrakenL2SwerveModule("Back Left Module",
            ConfigMap.BACK_LEFT_MODULE_DRIVE_MOTOR,
            ConfigMap.BACK_LEFT_MODULE_STEER_MOTOR, ConfigMap.BACK_LEFT_MODULE_STEER_ENCODER,
            ConfigMap.BACK_LEFT_MODULE_STEER_OFFSET);
    private final SwerveModule mBackRight = new KrakenL2SwerveModule("Back Right Module",
            ConfigMap.BACK_RIGHT_MODULE_DRIVE_MOTOR,
            ConfigMap.BACK_RIGHT_MODULE_STEER_MOTOR, ConfigMap.BACK_RIGHT_MODULE_STEER_ENCODER,
            ConfigMap.BACK_RIGHT_MODULE_STEER_OFFSET);

    // Odometry & Kinematics
    private final SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(ConfigMap.FRONT_LEFT_MODULE_POSITION,
            ConfigMap.FRONT_RIGHT_MODULE_POSITION, ConfigMap.BACK_LEFT_MODULE_POSITION,
            ConfigMap.BACK_RIGHT_MODULE_POSITION);

    private final SwerveDrivePoseEstimator mPoseEstimator;
    private boolean mIsFieldOriented = true;
    private boolean mOdometryBeenSet = false;
    private boolean ignoreLimelightDistanceChecks = false;

    // Sensors
    private final AHRS mNavX = new AHRS(SPI.Port.kMXP);

    // Autonomous positioning
    private PathPlannerTrajectory mDesiredTrajectory = null;
    private WaypointHolder mDesiredWaypoint = null;
    private boolean mIsManual = true;
    private Timer trajectoryTimer;
    private PathPlannerPath mPath;
    private final EVector TRAJECTORY_PID = new EVector(3, 0, 0);
    private final PIDController mXController = new PIDController(TRAJECTORY_PID.x, TRAJECTORY_PID.y, TRAJECTORY_PID.z);
    private final PIDController mYController = new PIDController(TRAJECTORY_PID.x, TRAJECTORY_PID.y, TRAJECTORY_PID.z);
    private final ProfiledPIDController mRotController = new ProfiledPIDController(4, 0, 0,
            new TrapezoidProfile.Constraints(3.6 * Math.PI, 7.2 * Math.PI));
    private final PIDController mNoteAimController = new PIDController(4, 0, 0);

    private final EVector GO_TO_PID = EVector.newVector(0.1, 0, 0);
    private final double MAX_ACCELERATION = 8.0;
    private final ProfiledPIDController mXGoToController = new ProfiledPIDController(GO_TO_PID.x, GO_TO_PID.y, GO_TO_PID.z,
            new TrapezoidProfile.Constraints(ConfigMap.MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION));
    private final ProfiledPIDController mYGoToController = new ProfiledPIDController(GO_TO_PID.x, GO_TO_PID.y, GO_TO_PID.z,
            new TrapezoidProfile.Constraints(ConfigMap.MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION));
    private final ProfiledPIDController mRotGoToController = new ProfiledPIDController(0.01, 0, 0,
            new TrapezoidProfile.Constraints(6.28, Math.PI * 12));
    private final HolonomicDriveController mWaypointController = new HolonomicDriveController(mXController,
            mYController, mRotController);

    private HolonomicDriveController mTrajectoryController;

    private Pose2d mDesiredPosition = new Pose2d();

    public DriveSystem() {
        super("DriveSystem", DriveSystemState.DRIVE_MANUAL);
        mPoseEstimator = new SwerveDrivePoseEstimator(mKinematics, new Rotation2d(), getModulePositions(),
                new Pose2d());
        mRotController.enableContinuousInput(0, Math.PI * 2);
        mNavX.resetDisplacement();
        mRotGoToController.enableContinuousInput(0, Math.PI * 2);

        Logger.recordOutput("DriveSystem/NoteRequestedRotation", 0);
        Logger.recordOutput("DriveSystem/MatchesState", true);

        Logger.recordOutput("DriveSystem/DesiredVelocities", EVector.newVector().toPose2d());
        Logger.recordOutput("DriveSystem/DesiredVelocityMagnitude", 0.0);
    }

    public enum DriveSystemState implements Subsystem.SubsystemState {
        DRIVE_SETPOINT,
        DRIVE_NOTE,
        DRIVE_SEEK,
        DRIVE_TRAJECTORY,
        DRIVE_MANUAL,
        DRIVE_ROTATION,
        SYSID,
        DRIVE_TRAJECTORY_NOTE,
        DRIVE_TRAJECTORY_QUICK // generally more unstable because it doesn't wait for a path to finish before letting control system contiune
    }

    public void setDesiredState(SubsystemState desiredState) {
        super.setDesiredState(desiredState);
        mNoteAimController.reset();
    }

    @Override
    public void getToState() {
        Logger.recordOutput("DriveSystem/OdometryBeenSet", mOdometryBeenSet);
        Logger.recordOutput("DriveSystem/CurrentPose", getRobotPose());
        Logger.recordOutput("DriveSystem/SwerveStates", getModuleStates());
        Logger.recordOutput("DriveSystem/MatchesState", matchesDesiredState());
        Logger.recordOutput("DriveSystem/DesiredPosition", mDesiredPosition);
        Logger.recordOutput("DriveSystem/MatchesDesiredPosition", matchesPosition(mDesiredPosition));

        Logger.recordOutput("SysID/XVelocity", mNavX.getVelocityX());
        Logger.recordOutput("SysID/YVelocity", mNavX.getVelocityY());
        if (trajectoryTimer != null) {
            Logger.recordOutput("DriveSystem/TrajectoryTimer", trajectoryTimer.get());
        } else {
            Logger.recordOutput("DriveSystem/TrajectoryTimer", 0.00);
        }

        if (!mOdometryBeenSet && currentMode == OperatingMode.AUTONOMOUS) {
            return;
        }

        switch ((DriveSystemState) getState()) {
            case DRIVE_SEEK:

                break;
            case DRIVE_SETPOINT:
            case DRIVE_NOTE:
                goToWaypoint();
                break;
            case DRIVE_ROTATION:
                alignRotation();
                break;
            case DRIVE_TRAJECTORY_NOTE:
            case DRIVE_TRAJECTORY:
            case DRIVE_TRAJECTORY_QUICK:
                followTrajectory();
                break;
            case DRIVE_MANUAL:
                break;
            default:
                break;
        }
    }

    @Override
    public boolean matchesDesiredState() {
        switch ((DriveSystemState) getState()) {
            case DRIVE_SETPOINT:
            case DRIVE_NOTE:
                return matchesDesiredPosition() || controlSystem.hasNote();
            case DRIVE_ROTATION:
                return matchesRotation(mDesiredWaypoint.getPosition().z);
            case DRIVE_TRAJECTORY:
                return matchesPosition(mDesiredTrajectory.getEndState().getTargetHolonomicPose())
                        && (currentTime >= mDesiredTrajectory.getTotalTimeSeconds());
            case DRIVE_TRAJECTORY_NOTE:
            case DRIVE_TRAJECTORY_QUICK:
                return (matchesPosition(mDesiredTrajectory.getEndState().getTargetHolonomicPose())
                        && (currentTime >= mDesiredTrajectory.getTotalTimeSeconds())) || controlSystem.hasNote();
            case DRIVE_MANUAL:
                return true;
            default:
                return false;
        }
    }

    public boolean matchesPosition(Pose2d other) {
        final double TOLERANCE = 0.1;
        double dist = other.getTranslation().getDistance(getRobotPose().getTranslation());

        Logger.recordOutput("DriveSystem/DistanceFromDP", dist);
        return dist <= TOLERANCE;
    }

    public boolean matchesDesiredPosition() {
        if (mIsManual) {
            return true;
        }

        if (mDesiredWaypoint != null) {
            double dist = mDesiredWaypoint.getPosition().dist(EVector.fromPose(getRobotPose()));
            return dist < 0.15;
        }
        return false;
    }

    public boolean matchesRotation(double rotation) {
        final double TOLERANCE = 0.15;
        double currentRotation = getRobotAngle().getRadians();
        double dist = Math.abs(currentRotation - rotation);

        return dist <= TOLERANCE;
    }

    @Override
    public void init() {
        mFrontLeft.init();
        mFrontRight.init();
        mBackLeft.init();
        mBackRight.init();

        switch (currentMode) {
            case AUTONOMOUS:
                mNavX.reset();
                mNavX.zeroYaw();
                mIsFieldOriented = true;
                mOdometryBeenSet = false;
                Logger.recordOutput("DriveSystem/OdometryBeenSet", mOdometryBeenSet);
                setInitialLocalization();
                break;
            case TELEOP:
                mIsFieldOriented = true;
                mIsManual = true;

                setDesiredState(DriveSystemState.DRIVE_MANUAL);
                setInitialLocalization();
                break;
            case DISABLED:
                break;
            case SIMULATION:
                break;
            case TEST:
                setDesiredState(DriveSystemState.DRIVE_MANUAL);
                setInitialLocalization();
                
                Logger.recordOutput("SysID/XVelocity", mNavX.getVelocityX());
                Logger.recordOutput("SysID/YVelocity", mNavX.getVelocityY());
                break;
            default:
                break;
        }

        mRotController.enableContinuousInput(0, 2 * Math.PI);
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void genericPeriodic() {
        Logger.recordOutput("DriveSystem/XVelocity", mNavX.getVelocityX());
        Logger.recordOutput("DriveSystem/YVelocity", mNavX.getVelocityY());
        Logger.recordOutput("DriveSystem/XAcceleration", mNavX.getRawAccelX());
        Logger.recordOutput("DriveSystem/YAcceleration", mNavX.getRawAccelY());

        if (mOdometryBeenSet) {
            updateOdometry();
        } else if (!mOdometryBeenSet)  { //if (currentMode == OperatingMode.AUTONOMOUS) {
            setInitialLocalization();
            System.out.println("retrying...");
        }
    }

    // Auto
    public void setWaypoint(WaypointHolder waypoint) {
        mDesiredWaypoint = waypoint.copy();
        mDesiredTrajectory = null;
        mIsManual = false;

        Logger.recordOutput("DriveSystem/WaypointPose", mDesiredWaypoint.getPosition().toPose2d());

        mXController.reset();
        mYController.reset();
        mRotController.reset(getRobotAngle().getRadians());

        var currentPose = getRobotPose();
        mXGoToController.reset(currentPose.getX());
        mYGoToController.reset(currentPose.getY());
        mRotGoToController.reset(getRobotAngle().getRadians());
    }

    public void goToWaypoint() {
          if (mDesiredWaypoint == null || controlSystem.hasNote()) {
            return;
        }

        EVector desiredWaypointPosition = mDesiredWaypoint.getPosition();
        double rotation = desiredWaypointPosition.z;

        ChassisSpeeds desiredChassisSpeeds = mWaypointController.calculate(
                getRobotPose(),
                desiredWaypointPosition.toPose2d(),
                mDesiredWaypoint.getSpeedMetersPerSecond(),
                new Rotation2d(rotation));

        driveWithChassisSpeeds(desiredChassisSpeeds);
    }

    public void setTrajectoryFromPath(PathPlannerPath path) {
        mPath = path;
        if (mOdometryBeenSet && currentMode == OperatingMode.AUTONOMOUS) {
            mPath.replan(getRobotPose(), getChassisSpeeds());
        }

        PathPlannerTrajectory pathTrajectory = new PathPlannerTrajectory(mPath, new ChassisSpeeds(), getRobotAngle());

        setTrajectory(pathTrajectory);
    }

    public void setTrajectory(PathPlannerTrajectory trajectory) {
        mDesiredTrajectory = trajectory;
        PIDController xTrajectoryController;
        PIDController yTrajectoryController;
        ProfiledPIDController rotTrajectoryController;
        xTrajectoryController = new PIDController(0, 0, 0);
        yTrajectoryController = new PIDController(0, 0, 0);
        rotTrajectoryController = new ProfiledPIDController(5.5, 0.01, 0.01,
                new TrapezoidProfile.Constraints(6.28, 12.14));
        rotTrajectoryController.enableContinuousInput(0, 2 * Math.PI);
        mTrajectoryController = new HolonomicDriveController(xTrajectoryController, yTrajectoryController,
                rotTrajectoryController);

        mDesiredPosition = mDesiredTrajectory.getEndState().getDifferentialPose();

        mXController.reset();
        mYController.reset();
        mRotController.reset(getRobotAngle().getRadians());
         
        trajectoryTimer = new Timer();
        trajectoryTimer.reset();
        if (mOdometryBeenSet) {
            trajectoryTimer.start();
        }
        mIsManual = false;
    }

    public void followTrajectory() {
        if (mIsManual && mDesiredTrajectory == null) {
            return;
        }

        if (!mOdometryBeenSet) {
            trajectoryTimer.stop();
            trajectoryTimer.reset();
            return;
        } 
        trajectoryTimer.start();
        
        double distanceFromEndPose = getRobotPose().getTranslation().getDistance(mDesiredPosition.getTranslation());
        final double END_DIST_TOLERANCE = 2.5; // in meters when we start seeking a note
        final double seenNoteOffset = Limelight.getTX(ConfigMap.NOTE_LIMELIGHT); // if 0.0 then no note seen
        final double noteAreaInView = Limelight.getTA(ConfigMap.NOTE_LIMELIGHT); // % of the area of the camera's output
        final boolean seesNote = seenNoteOffset != 0.0;
        final boolean noteTooSmall = noteAreaInView < .1;
        final boolean noteTooFar = Math.abs(seenNoteOffset) > 18;

        double newYVelocity = 0.0;
        Logger.recordOutput("DriveSystem/NoteOffest", seenNoteOffset);

        //CONVERSION OF A pathPlannerState INTO desiredChassisSpeeds at a given time in the trajectory
        PathPlannerTrajectory.State pathplannerState = mDesiredTrajectory.sample(trajectoryTimer.get());
        Pose2d targetPose = new Pose2d(pathplannerState.positionMeters, pathplannerState.heading);
        Rotation2d targetOrientation = EEGeometryUtil.normalizeAngle(pathplannerState.getTargetHolonomicPose().getRotation());

        ChassisSpeeds desiredChassisSpeeds = mTrajectoryController.calculate(
                getRobotPose(),
                targetPose,
                pathplannerState.velocityMps,
                targetOrientation);
                Logger.recordOutput("DriveSystem/TargetTrajectory", targetPose);

        Logger.recordOutput("DriveSystem/SeesNote", seesNote);
        Logger.recordOutput("DriveSystem/NoteTooSmall", noteTooSmall);
        Logger.recordOutput("DriveSystem/NoteTooFar", noteTooFar);
        
        if (getState() == DriveSystemState.DRIVE_TRAJECTORY_NOTE && distanceFromEndPose < END_DIST_TOLERANCE && seesNote && !noteTooSmall && !noteTooFar) {
            controlSystem.LEDsSeesNote();
            final double kP = .1; //super low for testing
            int sideFlip = ControlSystem.isRed() ? -1 : 1;
            newYVelocity = seenNoteOffset * kP * sideFlip;

            Logger.recordOutput("DriveSystem/SideFlip", sideFlip);
            Logger.recordOutput("DriveSystem/NoteRequestedVelocity", newYVelocity);
            Logger.recordOutput("DriveSystem/AdjustingToNote", true);
            desiredChassisSpeeds.vyMetersPerSecond = newYVelocity;
        }

        driveWithChassisSpeeds(desiredChassisSpeeds);
    }

    //waypoint solution
    private WaypointHolder createWayPointToSeenNote(double noteOffset, Pose2d originalTargetPose) {
        Rotation2d rotToNote = Rotation2d.fromDegrees(noteOffset);
        double distanceFromNote = getRobotPose().getTranslation().getDistance(originalTargetPose.getTranslation());
        Logger.recordOutput("DriveSystem/SeenNoteOffset", noteOffset);
        Logger.recordOutput("DriveSystem/DistanceFromNote", distanceFromNote);
        //ASSUMING WE ARE FACING ALMOST 0 OR 180 ON THE FIELD GOING FOR A NOTE, ADJUST Y coordinate of targetPose
        double targetY = originalTargetPose.getY() - Math.tan(rotToNote.getRadians())*distanceFromNote * Math.cos(originalTargetPose.getRotation().getRadians());
        double targetX = originalTargetPose.getX();
        Rotation2d targetRotation = originalTargetPose.getRotation().minus(rotToNote);
        return new WaypointHolder(targetX, targetY, targetRotation.getRadians(), AutoStep.DEFAULT_SPEED);
    }

    public void setRotation(double rotationRads) {
        Pose2d currentPose = getRobotPose();
        mDesiredWaypoint = new WaypointHolder(currentPose.getX(), currentPose.getY(), rotationRads, 0);
        mDesiredTrajectory = null;
        mIsManual = false;

        setWaypoint(mDesiredWaypoint);

    }

    private void alignRotation() {
        if (matchesRotation(mDesiredWaypoint.getPosition().z)) {
            driveRaw(0, 0, 0);
            return;
        }

        double rotRPS = mRotController.calculate(getRobotAngle().getRadians(), mDesiredWaypoint.getPosition().z);
        driveRaw(0, 0, rotRPS);
    }

    //Driving
    public void driveWithChassisSpeeds(ChassisSpeeds speeds) {
        if (getState() == DriveSystemState.DRIVE_MANUAL)
            return;

        SwerveModuleState[] moduleStates = mKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, ConfigMap.MAX_VELOCITY_METERS_PER_SECOND);

        mFrontLeft.runDesiredModuleState(moduleStates[0]);
        mFrontRight.runDesiredModuleState(moduleStates[1]);
        mBackLeft.runDesiredModuleState(moduleStates[2]);
        mBackRight.runDesiredModuleState(moduleStates[3]);
    }

    public void driveRaw(double xSpeed, double ySpeed, double rot) {
        SwerveModuleState[] moduleStates = mKinematics.toSwerveModuleStates(
                mIsFieldOriented
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
                                xSpeed,
                                ySpeed,
                                rot),
                        getRobotAngle())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, ConfigMap.MAX_VELOCITY_METERS_PER_SECOND);
        Logger.recordOutput("DriveSystem/CurrentPose", getRobotPose());
        Logger.recordOutput("DriveSystem/DesiredVelocities", EVector.newVector(xSpeed, ySpeed, rot).toPose2d());
        Logger.recordOutput("DriveSystem/DesiredVelocity Magnitude",
                EVector.newVector(xSpeed, ySpeed, rot).magnitude());

        mFrontLeft.runDesiredModuleState(moduleStates[0]);
        mFrontRight.runDesiredModuleState(moduleStates[1]);
        mBackLeft.runDesiredModuleState(moduleStates[2]);
        mBackRight.runDesiredModuleState(moduleStates[3]);
    }

    // Drive Info
    public Rotation2d getRobotAngle() {
        return mPoseEstimator.getEstimatedPosition().getRotation();
    }

    private Rotation2d getNavXAngle() {
        return new Rotation2d(-mNavX.getRotation2d().getRadians());
    }

    public Pose2d getRobotPose() {
        return new Pose2d(mPoseEstimator.getEstimatedPosition().getTranslation(), getRobotAngle());
    }

    public ChassisSpeeds getChassisSpeeds() {
        return mKinematics.toChassisSpeeds(getModuleStates());
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                mFrontLeft.getModulePosition(),
                mFrontRight.getModulePosition(),
                mBackLeft.getModulePosition(),
                mBackRight.getModulePosition()
        };
    }

    private SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                mFrontLeft.getCurrentState(),
                mFrontRight.getCurrentState(),
                mBackLeft.getCurrentState(),
                mFrontLeft.getCurrentState()
        };
    }

    //Localization / Odometry
    private void setInitialLocalization() {
        if (!mOdometryBeenSet && Limelight.getTV(ConfigMap.APRILTAG_LIMELIGHT) == 1) {

            Pose2d limelightPose = Limelight.getBotPose2d(ConfigMap.APRILTAG_LIMELIGHT);
            mPoseEstimator.resetPosition(getNavXAngle(), getModulePositions(), limelightPose);
            mOdometryBeenSet = true;
        } else if (!mOdometryBeenSet && currentMode == OperatingMode.AUTONOMOUS) {
            try {
                Pose2d firstPose = controlSystem.getAutoStartingPose2d();

                mPoseEstimator.resetPosition(getNavXAngle(), getModulePositions(), firstPose);
                mOdometryBeenSet = true;
            } catch (Exception e) {
                System.err.println(e);
            }
        } else if (!mOdometryBeenSet) {
             //Just set to 0,0 with a rotation respective to it's alliance color
             double startingDegRotation = ControlSystem.isRed() ? 180 : 0;
             mPoseEstimator.resetPosition(new Rotation2d(startingDegRotation), getModulePositions(), new Pose2d());

             // wait for Limelight
             ignoreLimelightDistanceChecks = true;
             mOdometryBeenSet = true;
        }
    }

    public void setInitialLocalization(Pose2d pose) {
        mPoseEstimator.resetPosition(getNavXAngle(), getModulePositions(), pose);
    }

    public void updateVisionLocalization(Pose2d visionEstimate) {
        if (!mOdometryBeenSet) {
            setInitialLocalization(visionEstimate);
        }

        double dist = getRobotPose().getTranslation().getDistance(visionEstimate.getTranslation());

        if (Math.abs(dist) >= 2 || visionEstimate.getX() == -99.9) {
            return;
        }

        //ignoreLimelightDistanceChecks = false;
        mPoseEstimator.addVisionMeasurement(visionEstimate, Timer.getFPGATimestamp());
    }

    private void updateOdometry() {
        mPoseEstimator.update(getNavXAngle(), getModulePositions());

    }

    public boolean hasOdometryBeenSet() {
        return mOdometryBeenSet;
    }

    public void zeroNavX() {
        mNavX.setAngleAdjustment(0);
        mNavX.zeroYaw();
        mPoseEstimator.resetPosition(getNavXAngle(), getModulePositions(),
                new Pose2d(getRobotPose().getTranslation(), new Rotation2d()));
    }

    public boolean badRoll() {
        double roll = mNavX.getRoll();
        // roll is from 180 to -180, i want it to be from 0 to 360
        if (roll < 0) {
            roll += 360;
        }

        roll -= 180;

        return Math.abs(roll) > 15;
    }
}
