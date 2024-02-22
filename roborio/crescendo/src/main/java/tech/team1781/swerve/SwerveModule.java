package tech.team1781.swerve;


import com.ctre.phoenix6.configs.CANcoderConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import tech.team1781.utils.SwerveModuleConfiguration;

public abstract class SwerveModule {

    public SwerveModule(int driveMotorID, int turnMotorID, int cancoderId, double cancoderOffset) {

    }

    protected SwerveModuleState desiredState; 

    public void init() {
        
    }

    public abstract Rotation2d getAbsoluteAngle();

    public abstract SwerveModulePosition getModulePosition();

    public abstract SwerveModuleState getCurrentState();

    public abstract void setDesiredState(SwerveModuleState sentDesiredState);

    abstract void syncRelativeToAbsoluteEncoder();

    static SwerveModuleConfiguration moduleConfiguration() {
        throw new UnsupportedOperationException();
    }

    private CANcoderConfiguration absoluteEncoderConfiguration(double offset) {
        throw new UnsupportedOperationException();
    }
}