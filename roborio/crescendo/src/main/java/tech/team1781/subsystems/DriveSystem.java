package tech.team1781.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import tech.team1781.ConfigMap;
import tech.team1781.swerve.NEOL1SwerveModule;
import tech.team1781.swerve.SwerveModule;
import tech.team1781.utils.EVector;

public class DriveSystem extends Subsystem {

    //Swerve Modules
    private final SwerveModule mFrontLeft = new NEOL1SwerveModule(ConfigMap.FRONT_LEFT_MODULE_DRIVE_MOTOR, ConfigMap.FRONT_LEFT_MODULE_STEER_MOTOR, ConfigMap.FRONT_LEFT_MODULE_STEER_ENCODER, ConfigMap.FRONT_LEFT_MODULE_STEER_OFFSET);
    private final SwerveModule mFrontRight = new NEOL1SwerveModule(ConfigMap.FRONT_RIGHT_MODULE_DRIVE_MOTOR, ConfigMap.FRONT_RIGHT_MODULE_STEER_MOTOR, ConfigMap.FRONT_RIGHT_MODULE_STEER_ENCODER, ConfigMap.FRONT_RIGHT_MODULE_STEER_OFFSET);
    private final SwerveModule mBackLeft = new NEOL1SwerveModule(ConfigMap.BACK_LEFT_MODULE_DRIVE_MOTOR, ConfigMap.BACK_LEFT_MODULE_STEER_MOTOR, ConfigMap.BACK_LEFT_MODULE_STEER_ENCODER, ConfigMap.BACK_LEFT_MODULE_STEER_OFFSET);
    private final SwerveModule mBackRight = new NEOL1SwerveModule(ConfigMap.BACK_RIGHT_MODULE_DRIVE_MOTOR, ConfigMap.BACK_RIGHT_MODULE_STEER_MOTOR, ConfigMap.BACK_RIGHT_MODULE_STEER_ENCODER, ConfigMap.BACK_RIGHT_MODULE_STEER_OFFSET);

    //Odometry & Kinematics
    private SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(ConfigMap.FRONT_LEFT_MODULE_POSITION,
            ConfigMap.FRONT_RIGHT_MODULE_POSITION, ConfigMap.BACK_LEFT_MODULE_POSITION, ConfigMap.BACK_RIGHT_MODULE_POSITION);
    private GenericEntry mXEntry = ConfigMap.SHUFFLEBOARD_TAB.add("X Position", 0).getEntry();
    private GenericEntry mYEntry = ConfigMap.SHUFFLEBOARD_TAB.add("Y Position", 0).getEntry();
    private GenericEntry mRotEntry = ConfigMap.SHUFFLEBOARD_TAB.add("Rotate Position", 0).getEntry();

    private SwerveDriveOdometry mOdometry;
    private boolean mIsFieldOriented = true;
    private double mNavXOffset = 0;
    private boolean mHasNavXOffsetBeenSet = false;
    //Sensors
    private AHRS mNavX = new AHRS(SPI.Port.kMXP);


    //Autonomous positioning 
    private PathPlannerTrajectory mDesiredTrajectory = null;
    private EVector mDesiredPosition = null;
    private boolean mIsManual = true;

    private PIDController mXController = new PIDController(1, 0, 0);
    private PIDController mYController = new PIDController(1, 0, 0);
    private ProfiledPIDController mRotController = new ProfiledPIDController(1, 0, 0, 
    new TrapezoidProfile.Constraints(6.28, 3.14));

    private HolonomicDriveController mTrajectoryController = new HolonomicDriveController(mXController, mYController, mRotController);

    public DriveSystem() {
        super("Drive System", DriveSystemState.DRIVE_MANUAL);

        mOdometry = new SwerveDriveOdometry(mKinematics, getRobotAngle(), getModulePositions());

        mRotController.enableContinuousInput(0, Math.PI * 2);

    }

    public enum DriveSystemState implements Subsystem.SubsystemState {
        DRIVE_SETPOINT, 
        DRIVE_TRAJECTORY,
        DRIVE_MANUAL
    } 

    @Override
    public void getToState() {
        switch((DriveSystemState) getState()) {
            case DRIVE_SETPOINT:
                goTo(mDesiredPosition);
            break;
            case DRIVE_TRAJECTORY:
                followTrajectory();
            break;
            case DRIVE_MANUAL:
                if(super.currentMode == OperatingMode.AUTONOMOUS) {
                    driveRaw(0, 0, 0);
                }
            break;
            default:
            break;
        }
    }

    @Override
    public boolean matchesDesiredState() {

        switch((DriveSystemState) getState()) {
            case DRIVE_SETPOINT:
            // return matchesDesiredPosition();
            return false;
            case DRIVE_TRAJECTORY:
            return false;
            // return mDesiredTrajectory.getTotalTimeSeconds() < currentTime;
            case DRIVE_MANUAL:
            return false;
            // return mIsManual;
            default: 
            return true;
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
        updateOdometry();
    }

    @Override
    public void init() {
        mRotController.reset(0);
        mFrontLeft.init();
        mFrontRight.init();
        mBackLeft.init();
        mBackRight.init();
        mNavX.reset();
        mNavX.zeroYaw();

        switch(currentMode) {
            case AUTONOMOUS:
                mIsFieldOriented = true;
                mHasNavXOffsetBeenSet = false;
            break;
            case TELEOP:
                mIsManual = true;
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
    }

    public void setOdometry(Pose2d pose) {
        mOdometry.resetPosition(getRobotAngle(), getModulePositions(), pose);
    }

    public void setNavXOffset(Rotation2d offset) {
        if (mHasNavXOffsetBeenSet) {
            return;
        }

        mNavXOffset = offset.getRadians();
        System.out.printf("navx offset set to %.2f\n", mNavXOffset);
        mHasNavXOffsetBeenSet = true;
    } 

    public void zeroNavX() {
        mNavX.setAngleAdjustment(0);
        mNavXOffset = 0.0;
        mHasNavXOffsetBeenSet = true;
        mNavX.zeroYaw();
    }

    public void followTrajectory() {
        if(mIsManual && mDesiredTrajectory == null) {
            return;
        }

        var pathplannerState = mDesiredTrajectory.sample(currentTime);
        edu.wpi.first.math.trajectory.Trajectory.State wpilibstate = new edu.wpi.first.math.trajectory.Trajectory.State(
            currentTime, 
            pathplannerState.velocityMps, 
            pathplannerState.accelerationMpsSq, 
            pathplannerState.getTargetHolonomicPose(), 
            pathplannerState.curvatureRadPerMeter);
            ChassisSpeeds desiredChassisSpeeds = mTrajectoryController.calculate(getRobotPose(), wpilibstate, 
            pathplannerState.getTargetHolonomicPose().getRotation());
        driveWithChassisSpeds(desiredChassisSpeeds);
        //System.out.println( EVector.fromPose(pathplannerState.getTargetHolonomicPose()).asString()+"=holonomic pose");
        // System.out.println("pos dist: " + EVector.fromPose(pathplannerState.getTargetHolonomicPose()).dist(EVector.fromPose(getRobotPose())));
    }

    public void goTo(EVector target) {
        if(mIsManual && mDesiredPosition == null) {
            return;
        }
        final double DISTANCE_TOLERANCE = 0.1;
        EVector robotPose = EVector.fromPose(getRobotPose());
        double diff = target.dist(robotPose);
        if(diff < DISTANCE_TOLERANCE) {
            return;
        }

        double xDutyCycle = mXController.calculate(robotPose.x, target.x);
        double yDutyCycle = mYController.calculate(robotPose.y, target.y);
        double rotDutyCycle = mRotController.calculate(robotPose.z, target.z);

        driveRaw(xDutyCycle, yDutyCycle, rotDutyCycle);
    }

    public void setTrajectory(PathPlannerTrajectory trajectory) {
        Pose2d initialPose = trajectory.getInitialTargetHolonomicPose();
        mDesiredTrajectory = trajectory;

        //this is probably not work
        //setNavXOffset(initialPose.getRotation());
        //hard coding 90 degrees to test
        setNavXOffset(new Rotation2d(0));
        setOdometry(initialPose);
        mDesiredPosition = null;
        mIsManual = false;
    }

    public void setTrajectoryFromPath(PathPlannerPath path) {
        PathPlannerTrajectory pathTajectory = new PathPlannerTrajectory(path, new ChassisSpeeds(), getRobotAngle());
        setTrajectory(pathTajectory);
    }

    public void setPosition(EVector position) {
        mDesiredPosition = position;
        mDesiredTrajectory = null;
        mIsManual = false;
    }

    public void driveWithChassisSpeds(ChassisSpeeds speeds) {
        if(getState() == DriveSystemState.DRIVE_MANUAL)
            return;
        SwerveModuleState[] moduleStates = mKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, ConfigMap.MAX_VELOCITY_METERS_PER_SECOND);

        mFrontLeft.setDesiredState(moduleStates[0]);
        mFrontRight.setDesiredState(moduleStates[1]);
        mBackLeft.setDesiredState(moduleStates[2]);
        mBackRight.setDesiredState(moduleStates[3]);
    }

    public void driveRaw(double xSpeed, double ySpeed, double rot) {
        SwerveModuleState[] moduleStates = mKinematics.toSwerveModuleStates(
            mIsFieldOriented ? 
                ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(xSpeed, ySpeed, rot), getRobotAngle()) 
                : new ChassisSpeeds(xSpeed, ySpeed, rot)
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, ConfigMap.MAX_VELOCITY_METERS_PER_SECOND);

        mFrontLeft.setDesiredState(moduleStates[0]);
        mFrontRight.setDesiredState(moduleStates[1]);
        mBackLeft.setDesiredState(moduleStates[2]);
        mBackRight.setDesiredState(moduleStates[3]);
    }

    public Rotation2d getRobotAngle() {
        double reportedVal = mNavX.getRotation2d().getRadians() + mNavXOffset;

        reportedVal %= 2 * Math.PI;
        if(reportedVal < 0) {
            reportedVal += 2 * Math.PI;
        }

        System.out.printf("navx reported angle radians: %.2f\n", reportedVal);
        return new Rotation2d(reportedVal);
    }

    public Pose2d getRobotPose() {
        return mOdometry.getPoseMeters();
    }

    public boolean matchesDesiredPosition() {
        if(mIsManual) {
            return true;
        }
        if(mDesiredPosition != null) {
            return mDesiredPosition.dist(EVector.fromPose(getRobotPose())) < 0.1;
        }
        return false;
    }

    private void updateOdometry() {
        mOdometry.update(getRobotAngle(), getModulePositions());
        
        Pose2d robotPose = getRobotPose();
        mXEntry.setDouble(robotPose.getX());
        mYEntry.setDouble(robotPose.getY());
        mRotEntry.setDouble(getRobotAngle().getRadians());
        //System.out.println(EVector.fromPose(robotPose).asString());
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            mFrontLeft.getModulePosition(),
            mFrontRight.getModulePosition(),
            mBackLeft.getModulePosition(),
            mBackRight.getModulePosition()
        };
    }

    
}
