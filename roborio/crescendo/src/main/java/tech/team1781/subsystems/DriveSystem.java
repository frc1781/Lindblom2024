package tech.team1781.subsystems;

public class DriveSystem extends EESubsystem{

    public DriveSystem() {
<<<<<<< Updated upstream
        super("Drive System");
        //TODO Auto-generated constructor stub
=======
        super("Drive System", DriveSystemState.DRIVE_MANUAL);
        mOdometry = new SwerveDriveOdometry(mKinematics, getRobotAngle(), getModulePositions());
        mRotController.enableContinuousInput(0, 2 * Math.PI);
>>>>>>> Stashed changes
    }

    public enum DriveSystemState implements EESubsystem.SubsystemState {
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

<<<<<<< Updated upstream
    @Override
    public void autonomousInit() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'autonomousInit'");
    }

=======
>>>>>>> Stashed changes
    @Override
    public void autonomousPeriodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'autonomousPeriodic'");
    }

    @Override
    public void teleopInit() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'teleopInit'");
    }
    int counter = 0;
    @Override
    public void teleopPeriodic() {
<<<<<<< Updated upstream
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'teleopPeriodic'");
    }
=======
        counter++;
        var frontRightModule = (NEOL1SwerveModule) mFrontLeft;
        if(counter % 120 == 0) {
          frontRightModule.printDesiredRadians();
        }    
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

        switch(currentMode) {
            case AUTONOMOUS:
            break;
            case TELEOP:
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
            new ChassisSpeeds(xSpeed, ySpeed, rot)
        );

        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, ConfigMap.MAX_VELOCITY_METERS_PER_SECOND);
        
        // System.err.printf("Front Left: %.2f, Front Right %.2f, Back Left: %.2f, Back Right: %.2f \n", moduleStates[0].angle.getRadians(), moduleStates[1].angle.getRadians(), moduleStates[2].angle.getRadians(), moduleStates[3].angle.getRadians());
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
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            mFrontLeft.getModulePosition(),
            mFrontRight.getModulePosition(),
            mBackLeft.getModulePosition(),
            mBackRight.getModulePosition()
        };
    }

>>>>>>> Stashed changes
    
}
