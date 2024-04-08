package tech.team1781.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import tech.team1781.ConfigMap;
import tech.team1781.ShuffleboardStyle;
import tech.team1781.autonomous.WaypointHolder;
import tech.team1781.control.ControlSystem;
import tech.team1781.swerve.KrakenL2SwerveModule;
import tech.team1781.swerve.NEOL1SwerveModule;
import tech.team1781.swerve.SwerveModule;
import tech.team1781.utils.EEGeometryUtil;
import tech.team1781.utils.EVector;
import tech.team1781.utils.Limelight;
import tech.team1781.utils.NetworkLogger;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

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
    private SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(ConfigMap.FRONT_LEFT_MODULE_POSITION,
            ConfigMap.FRONT_RIGHT_MODULE_POSITION, ConfigMap.BACK_LEFT_MODULE_POSITION,
            ConfigMap.BACK_RIGHT_MODULE_POSITION);

    private SwerveDrivePoseEstimator mPoseEstimator;
    private boolean mIsFieldOriented = true;
    private double mNavXOffset = 0;
    private boolean mHasNavXOffsetBeenSet = false;
    private boolean mOdometryBeenSet = false;
    // Sensors
    private AHRS mNavX = new AHRS(SPI.Port.kMXP);

    // Autonomous positioning
    private PathPlannerTrajectory mDesiredTrajectory = null;
    private WaypointHolder mDesiredWaypoint = null;
    private boolean mIsManual = true;

    private final EVector TRAJECTORY_PID = new EVector(3, 0, 0);
    private PIDController mXController = new PIDController(TRAJECTORY_PID.x, TRAJECTORY_PID.y, TRAJECTORY_PID.z);
    private PIDController mYController = new PIDController(TRAJECTORY_PID.x, TRAJECTORY_PID.y, TRAJECTORY_PID.z);
    private ProfiledPIDController mRotController = new ProfiledPIDController(4, 0, 0,
            new TrapezoidProfile.Constraints(3.6 * Math.PI, 7.2 * Math.PI));
    private PIDController mNoteAimController = new PIDController(1, 0, 0);

    private final EVector GO_TO_PID = EVector.newVector(0.1, 0, 0);
    private final double MAX_ACCELERATION = 8.0;
    private ProfiledPIDController mXGoToController = new ProfiledPIDController(GO_TO_PID.x, GO_TO_PID.y, GO_TO_PID.z,
            new TrapezoidProfile.Constraints(ConfigMap.MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION));
    private ProfiledPIDController mYGoToController = new ProfiledPIDController(GO_TO_PID.x, GO_TO_PID.y, GO_TO_PID.z,
            new TrapezoidProfile.Constraints(ConfigMap.MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION));
    private ProfiledPIDController mRotGoToController = new ProfiledPIDController(0.01, 0, 0,
            new TrapezoidProfile.Constraints(6.28, Math.PI * 12));
    private HolonomicDriveController mWaypointController = new HolonomicDriveController(mXController,
            mYController, mRotController);

    private HolonomicDriveController mTrajectoryController;

    private GenericEntry mRobotXEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "X Position", -1,
            ShuffleboardStyle.ROBOT_X);
    private GenericEntry mRobotYEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Y Position", -1,
            ShuffleboardStyle.ROBOT_Y);
    private GenericEntry mRobotThetaEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Theta", -1,
            ShuffleboardStyle.ROBOT_THETA);
    private GenericEntry mRobotXSpeedEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "X Speed", -1,
            ShuffleboardStyle.ROBOT_X_VELOCITY);
    private GenericEntry mRobotYSpeedEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Y Speed", -1,
            ShuffleboardStyle.ROBOT_Y_VELOCITY);
    private GenericEntry mRobotVelocityEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB,
            "Overall Velocity", -1, ShuffleboardStyle.ROBOT_VELOCITY);

    private Field2d mField = new Field2d();

    public DriveSystem() {
        super("Drive System", DriveSystemState.DRIVE_MANUAL);
        // mOdometry = new SwerveDriveOdometry(mKinematics, getRobotAngle(),
        // getModulePositions());
        mPoseEstimator = new SwerveDrivePoseEstimator(mKinematics, new Rotation2d(), getModulePositions(),
                new Pose2d());
        mRotController.enableContinuousInput(0, Math.PI * 2);
        mNavX.resetDisplacement();

        ConfigMap.SHUFFLEBOARD_TAB.add("Field", mField)
                .withPosition((int) ShuffleboardStyle.ROBOT_POSITION_FIELD.position.x,
                        (int) ShuffleboardStyle.ROBOT_POSITION_FIELD.position.y)
                .withSize((int) ShuffleboardStyle.ROBOT_POSITION_FIELD.size.x,
                        (int) ShuffleboardStyle.ROBOT_POSITION_FIELD.size.y);
        mRotGoToController.enableContinuousInput(0, Math.PI * 2);

        NetworkLogger.initLog("Note Aim Requested Rotation", 0);
        NetworkLogger.initLog("Drive System Matches State", true);

        NetworkLogger.initLog("Drive System Desired Velocities", EVector.newVector());
        NetworkLogger.initLog("Drive System Desired Velocity Magnitude", 0.0);
    }

    public enum DriveSystemState implements Subsystem.SubsystemState {
        DRIVE_SETPOINT,
        DRIVE_NOTE,
        DRIVE_TRAJECTORY,
        DRIVE_MANUAL,
        DRIVE_ROTATION,
        SYSID
    }

    @Override
    public void getToState() {
        // System.out.println(getState());
        switch ((DriveSystemState) getState()) {
            case DRIVE_SETPOINT:
            case DRIVE_NOTE:
                goToWaypoint();
                break;
            case DRIVE_ROTATION:
                alignRotation();
                break;
            case DRIVE_TRAJECTORY:
                var trajectoryInitialPose = mDesiredTrajectory.getInitialState().getTargetHolonomicPose();
                followTrajectory();
                break;
            case DRIVE_MANUAL:
                if (super.currentMode == OperatingMode.AUTONOMOUS) {
                    // driveRaw(0, 0, 0);
                }
                break;
            case SYSID:
                driveRaw(1, 0, 0);
                ChassisSpeeds currentSpeeds = getChassisSpeeds();
                System.out.print(super.currentTime + "," + currentSpeeds.vxMetersPerSecond + ","
                        + currentSpeeds.vyMetersPerSecond + "," + currentSpeeds.omegaRadiansPerSecond);
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
                return matchesDesiredPosition();
            case DRIVE_ROTATION:
                return matchesRotation(mDesiredWaypoint.getPosition().z);
            // return false;
            case DRIVE_TRAJECTORY:
                return matchesPosition(mDesiredTrajectory.getEndState().getTargetHolonomicPose())
                        && (currentTime >= mDesiredTrajectory.getTotalTimeSeconds());
            case DRIVE_MANUAL:
                return true;
            // case DRIVE_NOTE:
            // return true;
            // return mIsManual;
            default:
                return false;
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void genericPeriodic() {
        NetworkLogger.logData("Drive System Matches State", matchesDesiredState());

        updateOdometry();
        mRobotXEntry.setDouble(getRobotPose().getX());
        mRobotYEntry.setDouble(getRobotPose().getY());
        mRobotXSpeedEntry.setDouble(Math.abs(getChassisSpeeds().vxMetersPerSecond));
        mRobotYSpeedEntry.setDouble(Math.abs(getChassisSpeeds().vyMetersPerSecond));
        mRobotVelocityEntry.setDouble(
                EVector.newVector(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond)
                        .magnitude());
        mRobotThetaEntry.setDouble(getRobotAngle().getRadians());
        mField.setRobotPose(getRobotPose());
        // mField.setRobotPose(getRobotPose());
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
                mHasNavXOffsetBeenSet = false;
                mOdometryBeenSet = false;
                break;
            case TELEOP:
                mHasNavXOffsetBeenSet = false;
                mIsFieldOriented = true;
                mIsManual = true;

                setDesiredState(DriveSystemState.DRIVE_MANUAL);
                break;
            case DISABLED:
                break;
            case SIMULATION:
                break;
            case TEST:
                break;
            default:
                break;
        }

        mRotController.enableContinuousInput(0, 2 * Math.PI);
    }

    public void updateVisionLocalization(Pose2d visionEstimate) {
        var visionEstimateVector = EVector.fromPose2d(visionEstimate);
        var currentPose = EVector.fromPose2d(getRobotPose());

        visionEstimateVector.z = currentPose.z;

        if (Math.abs(currentPose.dist(visionEstimateVector)) >= 2 || visionEstimate.getX() == -99.9) {
            return;
        }

        mPoseEstimator.addVisionMeasurement(visionEstimateVector.toPose2d(), Timer.getFPGATimestamp());
    }

    public void setOdometry(Pose2d pose) {
        mPoseEstimator.resetPosition(getRobotAngle(), getModulePositions(), pose);
    }

    public void setNavXOffset(Rotation2d offset) {
        if (mHasNavXOffsetBeenSet) {
            return;
        }

        mNavXOffset = offset.getRadians();
        mPoseEstimator = new SwerveDrivePoseEstimator(mKinematics, getRobotAngle(), getModulePositions(),
                getRobotPose());
        // mPoseEstimator.resetPosition(getRobotAngle(), getModulePositions(), )
        mHasNavXOffsetBeenSet = true;
    }

    public void zeroNavX() {
        mNavX.setAngleAdjustment(0);
        mNavXOffset = 0.0;
        mHasNavXOffsetBeenSet = true;
        mNavX.zeroYaw();
    }

    public void followTrajectory() {
        if (mIsManual && mDesiredTrajectory == null) {
            return;
        }

        var pathplannerState = mDesiredTrajectory.sample(currentTime);
        mTrajectoryController.calculate(getRobotPose(), getRobotPose(), currentTime, getRobotAngle());
        ChassisSpeeds desiredChassisSpeeds = mTrajectoryController.calculate(
                getRobotPose(),
                new Pose2d(pathplannerState.positionMeters, pathplannerState.heading),
                pathplannerState.velocityMps,
                pathplannerState.getTargetHolonomicPose().getRotation());
        driveWithChassisSpeds(desiredChassisSpeeds);
    }

    public void goToWaypoint() {
        if (mDesiredWaypoint == null) {
            return;
        }

        EVector currentPoseVector = EVector.fromPose(getRobotPose());
        EVector desiredWaypointPosition = mDesiredWaypoint.getPosition();
        double rotation = desiredWaypointPosition.z;

        ChassisSpeeds desiredChassisSpeeds = mWaypointController.calculate(
                getRobotPose(),
                desiredWaypointPosition.toPose2d(),
                mDesiredWaypoint.getSpeedMetersPerSecond(),
                new Rotation2d(rotation));

        if (getState() == DriveSystemState.DRIVE_NOTE) {
            final double DISTANCE_TOLERANCE = 1.5;
            final double ANGLE_TOLERANCE = 0.1;

            double dist = currentPoseVector.withZ(0)
                    .dist(
                            desiredWaypointPosition.withZ(0));

            if (dist <= DISTANCE_TOLERANCE) {

                double xObservedNoteAngle = Math.toRadians(Limelight.getTX(ConfigMap.NOTE_LIMELIGHT));
                if (xObservedNoteAngle != 0.0) {
                    double currentAngle = getRobotAngle().getRadians();
                    if (Math.abs(xObservedNoteAngle) > ANGLE_TOLERANCE) {
                        desiredChassisSpeeds.omegaRadiansPerSecond = -mNoteAimController.calculate(xObservedNoteAngle, 0);
                        mDesiredWaypoint.changeRotation(currentAngle);
                    } else {
                        mDesiredWaypoint.changeX(
                                Math.cos(currentAngle) * dist
                                        + currentPoseVector.x);
                        mDesiredWaypoint.changeY(
                                Math.sin(currentAngle) * dist
                                        + currentPoseVector.y);
                    }
                }

            }

        }

        // System.out.println("Desired Pose: " + desiredWaypointPosition.withZ(rotation)
        // + " Current Pose: "
        // + EVector.fromPose2d(currentPose) + " X: " +
        // desiredChassisSpeeds.vxMetersPerSecond + " Y: " +
        // desiredChassisSpeeds.vyMetersPerSecond + " Rot: " +
        // desiredChassisSpeeds.omegaRadiansPerSecond);

        driveWithChassisSpeds(desiredChassisSpeeds);
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

    public void setTrajectory(PathPlannerTrajectory trajectory) {
        mDesiredTrajectory = trajectory;

        mDesiredWaypoint = null;
        mIsManual = false;
    }

    public void setTrajectoryFromPath(PathPlannerPath path) {
        Pose2d initialPose = path.getPreviewStartingHolonomicPose();
        Rotation2d startingOrientation = initialPose.getRotation();
        // also use current speed of robot
        if (!mOdometryBeenSet) {
            setNavXOffset(startingOrientation);
            setOdometry(initialPose);
            mXController.reset();
            mYController.reset();
            mRotController.reset(getRobotAngle().getRadians());
            mTrajectoryController = new HolonomicDriveController(mXController, mYController, mRotController);

            mOdometryBeenSet = true;
        }

        PathPlannerTrajectory pathTrajectory = new PathPlannerTrajectory(path, new ChassisSpeeds(), getRobotAngle());
        setTrajectory(pathTrajectory);
    }

    public void setWaypoint(WaypointHolder waypoint) {
        mDesiredWaypoint = waypoint.copy();
        mDesiredTrajectory = null;
        mIsManual = false;

        if (!mOdometryBeenSet) {
            setOdometry(mDesiredWaypoint.getPosition().toPose2d());
            setNavXOffset(Rotation2d.fromRadians(mDesiredWaypoint.getPosition().z));
            mOdometryBeenSet = true;
        }

        mXController.reset();
        mYController.reset();
        mRotController.reset(getRobotAngle().getRadians());

        var currentPose = getRobotPose();
        mXGoToController.reset(currentPose.getX());
        mYGoToController.reset(currentPose.getY());
        mRotGoToController.reset(getRobotAngle().getRadians());
    }

    public void setRotation(double rotationRads) {
        Pose2d currentPose = getRobotPose();
        mDesiredWaypoint = new WaypointHolder(currentPose.getX(), currentPose.getY(), rotationRads, 0);
        mDesiredTrajectory = null;
        mIsManual = false;

        setWaypoint(mDesiredWaypoint);

    }

    public void driveWithChassisSpeds(ChassisSpeeds speeds) {
        if (getState() == DriveSystemState.DRIVE_MANUAL)
            return;
        // System.out.println("Driving: " + speeds.vxMetersPerSecond + " " +
        // speeds.vyMetersPerSecond + " "
        // + speeds.omegaRadiansPerSecond);

        SwerveModuleState[] moduleStates = mKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, ConfigMap.MAX_VELOCITY_METERS_PER_SECOND);

        mFrontLeft.setDesiredState(moduleStates[0]);
        mFrontRight.setDesiredState(moduleStates[1]);
        mBackLeft.setDesiredState(moduleStates[2]);
        mBackRight.setDesiredState(moduleStates[3]);
    }

    public boolean matchesPosition(Pose2d other) {
        final double TOLERANCE = 0.1;
        EVector currentPose = EVector.fromPose(getRobotPose());
        EVector otherPose = EVector.fromPose(other);
        double dist = currentPose.dist(otherPose);
        return dist <= TOLERANCE;
    }

    public boolean matchesRotation(double rotation) {
        final double TOLERANCE = 0.15;
        double currentRotation = getRobotAngle().getRadians();
        double dist = Math.abs(currentRotation - rotation);

        return dist <= TOLERANCE;
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

        NetworkLogger.logData("Drive System Desired Velocities", EVector.newVector(xSpeed, ySpeed, rot));
        NetworkLogger.logData("Drive System Desired Velocity Magnitude",
                EVector.newVector(xSpeed, ySpeed, rot).magnitude());

        mFrontLeft.setDesiredState(moduleStates[0]);
        mFrontRight.setDesiredState(moduleStates[1]);
        mBackLeft.setDesiredState(moduleStates[2]);
        mBackRight.setDesiredState(moduleStates[3]);
    }

    public Rotation2d getRobotAngle() {
        double reportedVal = -mNavX.getRotation2d().getRadians() + mNavXOffset;

        reportedVal %= 2 * Math.PI;
        if (reportedVal < 0) {
            reportedVal += 2 * Math.PI;
        }

        return new Rotation2d(reportedVal);
    }

    public Pose2d getRobotPose() {
        return new Pose2d(mPoseEstimator.getEstimatedPosition().getTranslation(), getRobotAngle());
    }

    public ChassisSpeeds getChassisSpeeds() {
        return mKinematics.toChassisSpeeds(getModuleStates());
    }

    public boolean matchesDesiredPosition() {
        if (mIsManual) {
            return true;
        }

        if (mDesiredWaypoint != null) {
            double dist = mDesiredWaypoint.getPosition().dist(EVector.fromPose(getRobotPose()));
            return dist < 0.1;
        }
        return false;
    }

    public void setDesiredState(SubsystemState desiredState) {
        super.setDesiredState(desiredState);
        mNoteAimController.reset();
    }

    public void printModules() {
        // ((NEOL1SwerveModule) mFrontLeft).printModuleState();
        // ((NEOL1SwerveModule) mFrontRight).printModuleState();
        // ((NEOL1SwerveModule) mBackLeft).printModuleState();
        // ((NEOL1SwerveModule) mBackRight).printModuleState();
    }

    // if (mIsManual && mDesiredPosition == null) {
    // return;
    // }
    // EVector robotPose = EVector.fromPose(getRobotPose());
    // if (matchesDesiredPosition()) {
    // // System.out.println("matches position
    // // **********************************************");
    // driveRaw(0, 0, 0);
    // return;
    // }

    // double xMPS = clamp(mXGoToController.calculate(robotPose.x, target.x),
    // ConfigMap.MAX_VELOCITY_METERS_PER_SECOND);
    // double yMPS = clamp(mYGoToController.calculate(robotPose.y, target.y),
    // ConfigMap.MAX_VELOCITY_METERS_PER_SECOND);
    // double rotRPS = mRotGoToController.calculate(getRobotAngle().getRadians(),
    // target.z);
    // double xObservedNoteAngle =
    // Math.toRadians(Limelight.getTX(ConfigMap.NOTE_LIMELIGHT));

    // if (getState() == DriveSystemState.DRIVE_NOTE) {
    // final double DIST_TOLERANCE = 1.5;
    // double dist = robotPose.withZ(0).dist(target.withZ(0));
    // if (xObservedNoteAngle != 0.0 && dist < DIST_TOLERANCE) {
    // final double ALIGNMENT_TOLERANCE = 0.1;
    // rotRPS = mNoteAimController.calculate(xObservedNoteAngle, 0);
    // NetworkLogger.logData("Note Aim Requested Rotation", rotRPS);

    // if (Math.abs(xObservedNoteAngle) < ALIGNMENT_TOLERANCE) {
    // double distToSetpoint = robotPose.withZ(0).dist(target.withZ(0));
    // EVector desiredPos = EVector.newVector(
    // (Math.cos(getRobotAngle().getRadians()) * distToSetpoint) + robotPose.x,
    // (Math.sin(getRobotAngle().getRadians()) * distToSetpoint) + robotPose.y,
    // getRobotAngle().getRadians());
    // mDesiredPosition = desiredPos;
    // }
    // mDesiredPosition.z = getRobotAngle().getRadians();

    // }
    // }
    // driveRaw(xMPS, yMPS, rotRPS);
    // }

    private void alignRotation() {
        if (matchesRotation(mDesiredWaypoint.getPosition().z)) {
            driveRaw(0, 0, 0);
            return;
        }

        double rotRPS = mRotController.calculate(getRobotAngle().getRadians(), mDesiredWaypoint.getPosition().z);
        driveRaw(0, 0, rotRPS);
    }

    private void updateOdometry() {
        // mOdometry.update(getRobotAngle(), getModulePositions());
        mPoseEstimator.update(getRobotAngle(), getModulePositions());
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

    private static double clamp(double input, double clampVal) {
        double sig = Math.signum(input);
        double abs = Math.abs(input);

        if (abs >= clampVal) {
            return clampVal * sig;
        }

        return input;

    }

    private void followTrajectory(double time) {
        if (mIsManual && mDesiredTrajectory == null) {
            return;
        }

        var pathplannerState = mDesiredTrajectory.sample(time);

        ChassisSpeeds desiredChassisSpeeds = mTrajectoryController.calculate(
                getRobotPose(),
                new Pose2d(pathplannerState.positionMeters, pathplannerState.heading),
                pathplannerState.velocityMps,
                pathplannerState.getTargetHolonomicPose().getRotation());
        driveWithChassisSpeds(desiredChassisSpeeds);
    }

}
