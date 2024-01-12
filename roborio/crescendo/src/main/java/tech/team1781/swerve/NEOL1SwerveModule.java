package tech.team1781.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import tech.team1781.ConfigMap;
import tech.team1781.utils.SwerveModuleConfiguration;

public class NEOL1SwerveModule extends SwerveModule{
    private SwerveModuleState mDesiredState = new SwerveModuleState();

    private final CANSparkMax mDriveMotor;
    private final CANSparkMax mTurnMotor;
    private final SparkPIDController mDrivePID;
    private final SparkPIDController mTurnPID;
    // private final PIDController mTurnPID;
    
    private final RelativeEncoder mDriveEncoder;
    private final RelativeEncoder mTurnEncoder;
    private final CANcoder mTurnAbsoluteEncoder;

    public NEOL1SwerveModule(int driveMotorID, int turnMotorID, int cancoderID, double cancoderOffset) {
        super(driveMotorID, turnMotorID, cancoderID, cancoderOffset);
        mDriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        mTurnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        
        mDriveMotor.restoreFactoryDefaults();
        mTurnMotor.restoreFactoryDefaults();

        mDrivePID = mDriveMotor.getPIDController();
        mTurnPID = mTurnMotor.getPIDController();
        // mTurnPID = new PIDController(moduleConfiguration().turningP, moduleConfiguration().turningI, moduleConfiguration().turningD);
        mDriveEncoder = mDriveMotor.getEncoder();
        mTurnEncoder = mTurnMotor.getEncoder();
        
        mDrivePID.setFeedbackDevice(mDriveEncoder);
        mTurnPID.setFeedbackDevice(mTurnEncoder);


        mTurnAbsoluteEncoder = new CANcoder(cancoderID);
        StatusCode statusCode = mTurnAbsoluteEncoder.getConfigurator().apply(absoluteEncoderConfiguration(cancoderOffset));
        if(statusCode != StatusCode.OK) {
            DriverStation.reportError("Could not configure CANcoder with ID: " + cancoderID, false);
        }

        mDrivePID.setP(moduleConfiguration().drivingP);
        mDrivePID.setI(moduleConfiguration().drivingI);
        mDrivePID.setD(moduleConfiguration().drivingD);
        mDrivePID.setFF(moduleConfiguration().drivingFF);

        mTurnPID.setP(moduleConfiguration().turningP);
        mTurnPID.setI(moduleConfiguration().turningI);
        mTurnPID.setD(moduleConfiguration().turningD);

        mTurnEncoder.setPositionConversionFactor(moduleConfiguration().radiansPerRevolution);
        mTurnEncoder.setVelocityConversionFactor(moduleConfiguration().radiansPerSecond);

        mDriveEncoder.setPositionConversionFactor(moduleConfiguration().metersPerRevolution);
        mTurnPID.setFF(moduleConfiguration().turningFF);

        mDrivePID.setOutputRange(moduleConfiguration().minDrivingMotorVoltage, moduleConfiguration().maxDrivingMotorVoltage);
        mTurnPID.setOutputRange(moduleConfiguration().minTurningMotorVoltage, moduleConfiguration().maxTurningMotorVoltage);
        // mTurnPID.enableContinuousInput(0, Math.PI * 2);

        mDriveMotor.burnFlash();
        mTurnMotor.burnFlash();
        mTurnEncoder.setPosition(0);
        mDriveEncoder.setPosition(0);
    }

    public Rotation2d getAbsoluteAngle() {
        double reportedVal = mTurnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble();

        reportedVal = reportedVal % 1.0;
        if(reportedVal < 0) {
            reportedVal += 1.0;
        }

        return new Rotation2d(reportedVal * 2 * Math.PI);
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(mDriveEncoder.getVelocity(), getAbsoluteAngle());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(mDriveEncoder.getPosition(), getAbsoluteAngle());
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, new Rotation2d(mTurnEncoder.getPosition()));
        // mDrivePID.setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity);
        mTurnPID.setReference(2, ControlType.kPosition);
        // double turnDutyCycle = mTurnPID.calculate(mTurnEncoder.getPosition(), optimizedState.angle.getRadians());
        // mTurnMotor.set(turnDutyCycle / (Math.PI * 2));

        // System.out.println(desiredState.angle.getRadians() + " :: " + mTurnAbsoluteEncoder.getPosition());
        mDesiredState = optimizedState;

        syncRelativeToAbsoluteEncoder();
    }

    public void printDesiredRadians() {
        System.out.println("desired: " + mDesiredState.angle.getRadians() + " current: " + getAbsoluteAngle().getRadians());
        // System.out.println("angle: " + mTurnEncoder.getPosition() + " abs encoder: " + getAbsoluteAngle().getRadians());
    }

    void syncRelativeToAbsoluteEncoder() {
        if(mTurnEncoder.getVelocity() >= 0.5) {
            return;
        }

        double diff = getAbsoluteAngle().getRadians() - mTurnEncoder.getPosition();
        if(Math.abs(diff) > 0.02) {
            mTurnEncoder.setPosition(getAbsoluteAngle().getRadians());
        }

    }

    static SwerveModuleConfiguration moduleConfiguration() {
        SwerveModuleConfiguration ret_val = new SwerveModuleConfiguration();

        ret_val.metersPerRevolution = 0.102 * Math.PI * (14.0/50.0) * (25.0/19.0) * (15.0 / 45.0);
        ret_val.radiansPerRevolution = 2 * Math.PI * (14.0/50.0) * (10.0/60.0);
        ret_val.velocityConversion = ret_val.metersPerRevolution / 60.0;
        ret_val.radiansPerSecond = ret_val.radiansPerRevolution / 60.0;

        ret_val.drivingP = 0.1;
        ret_val.drivingI = 0.0;
        ret_val.drivingD = 0.01;
        ret_val.drivingFF = 1.0 / (ConfigMap.MAX_VELOCITY_METERS_PER_SECOND + 0.08);

        ret_val.turningP = 0.01; //0.01
        ret_val.turningI = 0.0;
        ret_val.turningD = 0.0; //0.3
        ret_val.turningFF = 0.0;

        ret_val.minDrivingMotorVoltage = -1;
        ret_val.maxDrivingMotorVoltage = 1;
        ret_val.minTurningMotorVoltage = -1;
        ret_val.maxTurningMotorVoltage = 1;

        return ret_val;
    }

    static CANcoderConfiguration absoluteEncoderConfiguration(double magnetOffset) {
        CANcoderConfiguration ret_val = new CANcoderConfiguration(); 

        ret_val.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        ret_val.MagnetSensor.MagnetOffset = magnetOffset;
        ret_val.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        return ret_val;
    }
}