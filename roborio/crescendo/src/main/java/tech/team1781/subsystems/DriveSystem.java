package tech.team1781.subsystems;


import com.kauailabs.navx.frc.AHRS;

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

    //Sensors
    private AHRS mNavX = new AHRS(SPI.Port.kMXP);


    //Autonomous positioning 
    private Trajectory mDesiredTrajectory = null;
    private EVector mDesiredPosition = null;
    private boolean mIsManual = true;

    private ProfiledPIDController mXController = new ProfiledPIDController(1.1, 0, 0.1, new TrapezoidProfile.Constraints(0, 0));
    private ProfiledPIDController mYController = new ProfiledPIDController(1.1, 0, 0.1, new TrapezoidProfile.Constraints(0, 0));
    private ProfiledPIDController mRotController = new ProfiledPIDController(2, 0, 0.1, new TrapezoidProfile.Constraints(0, 0));

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
            return mDesiredTrajectory.getTotalTimeSeconds() < currentTime;
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
        mXController.reset(0);
        mYController.reset(0);
        mRotController.reset(0);

        mFrontLeft.init();
        mFrontRight.init();
        mBackLeft.init();
        mBackRight.init();

        mNavX.reset();
        mNavX.setAngleAdjustment(0);
        mNavX.zeroYaw();

        switch(currentMode) {
            case AUTONOMOUS:
                zeroNavX();
                setOdometry(getRobotPose()); 
                mIsFieldOriented = true;
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

    public void zeroNavX() {
        mNavX.zeroYaw();
    }

    public void followTrajectory() {
        if(mIsManual && mDesiredTrajectory == null) {
            return;
        }

        Pose2d sampledPose = mDesiredTrajectory.sample(currentTime).poseMeters;
        goTo(EVector.fromPose(sampledPose));
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

    public void setTrajectory(Trajectory trajectory) {
        mDesiredTrajectory = trajectory;
        mDesiredPosition = null;
        mIsManual = false;
    }

    public void setPosition(EVector position) {
        mDesiredPosition = position;
        mDesiredTrajectory = null;
        mIsManual = false;
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
        double reportedVal = mNavX.getRotation2d().getRadians();

        reportedVal %= 2 * Math.PI;
        if(reportedVal < 0) {
            reportedVal += 2 * Math.PI;
        }

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
