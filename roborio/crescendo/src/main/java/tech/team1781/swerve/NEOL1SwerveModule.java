package tech.team1781.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;

public class NEOL1SwerveModule {
    private SwerveModuleState mDesiredState = new SwerveModuleState();

    private final CANSparkMax mDriveMotor;
    private final CANSparkMax mTurnMotor;
    private final SparkPIDController mDrivePID;
    private final SparkPIDController mTurnPID;
    
    private final RelativeEncoder mDriveEncoder;
    private final RelativeEncoder mTurnEncoder;
    private final CANcoder mTurnAbsoluteEncoder;

    public NEOL1SwerveModule(int driveMotorID, int turnMotorID, int cancoderID, double cancoderOffset) {
        mDriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        mTurnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        
        mDriveMotor.restoreFactoryDefaults();
        mTurnMotor.restoreFactoryDefaults();

        mDrivePID = mDriveMotor.getPIDController();
        mTurnPID = mTurnMotor.getPIDController();
        mDriveEncoder = mDriveMotor.getEncoder();
        mTurnEncoder = mDriveMotor.getEncoder();
        
        mDrivePID.setFeedbackDevice(mDriveEncoder);
        mTurnPID.setFeedbackDevice(mTurnEncoder);

        mTurnAbsoluteEncoder = new CANcoder(cancoderID);
        StatusCode statusCode = mTurnAbsoluteEncoder.getConfigurator().apply(absoluteEncoderConfiguration(cancoderOffset));
        if(statusCode != StatusCode.OK) {
            DriverStation.reportError("Could not configure CANcoder with ID: " + cancoderID, false);
        }

        
        


        

    }

    private CANcoderConfiguration absoluteEncoderConfiguration(double magnetOffset) {
        CANcoderConfiguration ret_val = new CANcoderConfiguration(); 

        ret_val.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        ret_val.MagnetSensor.MagnetOffset = magnetOffset;
        ret_val.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        return ret_val;
    }
}
