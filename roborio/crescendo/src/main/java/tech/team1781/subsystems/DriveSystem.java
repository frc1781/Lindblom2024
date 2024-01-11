package tech.team1781.subsystems;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import tech.team1781.ConfigMap;
import tech.team1781.swerve.NEOL1SwerveModule;
import tech.team1781.swerve.SwerveModule;

public class DriveSystem extends Subsystem{

    //Swerve Modules
    private final SwerveModule mFrontLeft = new NEOL1SwerveModule(0, 0, 0, 0);
    private final SwerveModule mFrontRight = new NEOL1SwerveModule(0, 0, 0, 0);
    private final SwerveModule mBackLeft = new NEOL1SwerveModule(0, 0, 0, 0);
    private final SwerveModule mBackRight = new NEOL1SwerveModule(0, 0, 0, 0);

    //Odometry & Kinematics
    private SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(ConfigMap.FRONT_LEFT_MODULE_POSITION,
            ConfigMap.FRONT_RIGHT_MODULE_POSITION, ConfigMap.BACK_LEFT_MODULE_POSITION, ConfigMap.BACK_RIGHT_MODULE_POSITION);

    private SwerveDriveOdometry mOdometry;

    //Sensors
    private AHRS mNavX = new AHRS(SPI.Port.kMXP);


    public DriveSystem() {
        super("Drive System");

        mOdometry = new SwerveDriveOdometry(mKinematics, getRobotAngle(), getModuleStates());
    }

    public enum DriveSystemState implements Subsystem.SubsystemState {
        DRIVE_SETPOINT, 
        DRIVE_TRAJECTORY,
        DRIVE_MANUAL
    } 

    @Override
    public void getToState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getToState'");
    }

    @Override
    public boolean matchesDesiredState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'matchesDesiredState'");
    }


    @Override
    public void autonomousPeriodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'autonomousPeriodic'");
    }

    @Override
    public void teleopPeriodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'teleopPeriodic'");
    }

    @Override
    public void genericPeriodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'genericPeriodic'");
    }

    @Override
    public void init() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'init'");
    }

    public void driveRaw(double xSpeed, double ySpeed, double rot) {
        SwerveModuleState[] moduleStates = mKinematics.toSwerveModuleStates(
            new ChassisSpeeds(xSpeed, ySpeed, rot)
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, ConfigMap.MAX_VELOCITY_METERS_PER_SECOND);

        mFrontLeft.setDesiredState(moduleStates[0]);
        mFrontRight.setDesiredState(moduleStates[1]);
        mBackLeft.setDesiredState(moduleStates[2]);
        mBackRight.setDesiredState(moduleStates[3]);
    }

    public Rotation2d getRobotAngle() {
        double reportedVal = mNavX.getAngle()/360 * 2 * Math.PI;

        reportedVal %= 2 * Math.PI;
        if(reportedVal < 0) {
            reportedVal += 2 * Math.PI;
        }

        return new Rotation2d(reportedVal);
    }

    public Pose2d getRobotPose() {
        return mOdometry.getPoseMeters();
    }

    private SwerveModulePosition[] getModuleStates() {
        return new SwerveModulePosition[]{
            mFrontLeft.getModulePosition(),
            mFrontRight.getModulePosition(),
            mBackLeft.getModulePosition(),
            mBackRight.getModulePosition()
        };
    }

    
}
