package tech.team1781.subsystems;

import java.util.ArrayList;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import tech.team1781.ConfigMap;
import tech.team1781.utils.EVector;

//EXAMPLE SUBSYSTEM, NOT FOR ACTUAL BOT
public class Scollector extends Subsystem {
    private CANSparkMax mCollectorMotor = new CANSparkMax(ConfigMap.COLLECTOR_MOTOR, CANSparkMax.MotorType.kBrushless);
    private CANSparkMax mRightShooterMotor = new CANSparkMax(ConfigMap.SHOOTER_RIGHT_MOTOR,
            CANSparkMax.MotorType.kBrushless);
    private CANSparkMax mLeftShooterMotor = new CANSparkMax(ConfigMap.SHOOTER_LEFT_MOTOR,
            CANSparkMax.MotorType.kBrushless);

    private final SparkPIDController mRightPID;
    private final SparkPIDController mLeftPID;

    private GenericEntry mThresholdEntry = ConfigMap.SHUFFLEBOARD_TAB.add("Shooter Threshold", 6).getEntry();

    private final EVector SHOOTER_PID = EVector.newVector(0.1, 0.0, 0.0);
    private final TrapezoidProfile.Constraints SHOOTER_CONSTRAINTS = new TrapezoidProfile.Constraints(9.0, 10);
    private ProfiledPIDController mLeftShooterPID = new ProfiledPIDController(SHOOTER_PID.x, SHOOTER_PID.y,
            SHOOTER_PID.z, SHOOTER_CONSTRAINTS);
    private ProfiledPIDController mRightShooterPID = new ProfiledPIDController(SHOOTER_PID.x, SHOOTER_PID.y,
            SHOOTER_PID.z, SHOOTER_CONSTRAINTS);

    private TimeOfFlight mNoteSensor = new TimeOfFlight(ConfigMap.SCOLLECTOR_TOF);

    private boolean mIsReadyToShoot = false;
    private boolean mArmInPosition = false;
    private boolean mHasShot = false;
    private Timer mShooterTimer = new Timer();

    public Scollector() {
        super("Scollector", ScollectorState.IDLE);
        mCollectorMotor.setIdleMode(IdleMode.kBrake);
        mLeftShooterMotor.setIdleMode(IdleMode.kCoast);
        mRightShooterMotor.setIdleMode(IdleMode.kCoast);

        final double conversionFactor = 0.100203 * 1 / 60;

        mLeftShooterMotor.getEncoder().setPositionConversionFactor(conversionFactor);
        mLeftShooterMotor.getEncoder().setVelocityConversionFactor(conversionFactor);
        mRightShooterMotor.getEncoder().setPositionConversionFactor(conversionFactor);
        mRightShooterMotor.getEncoder().setVelocityConversionFactor(conversionFactor);

        mRightPID = mRightShooterMotor.getPIDController();
        mLeftPID = mLeftShooterMotor.getPIDController();
        mRightPID.setFeedbackDevice(mRightShooterMotor.getEncoder());
        mLeftPID.setFeedbackDevice(mLeftShooterMotor.getEncoder());
        mRightPID.setP(0.5);
        mRightPID.setI(0);
        mRightPID.setD(0.01);
        mRightPID.setFF(1.0 / 9.8);

        mLeftPID.setP(0.5);
        mLeftPID.setI(0);
        mLeftPID.setD(0.01);
        mLeftPID.setFF(1.0 / 9.8);
    }

    public enum ScollectorState implements SubsystemState {
        IDLE, COLLECT, SPIT, SHOOT, COLLECT_RAMP, COLLECT_AUTO_SHOOT, RAMP_SHOOTER, SEND_NOTE_SHOOT
    }

    @Override
    public void genericPeriodic() {
    }

    @Override
    public void init() {
        mIsReadyToShoot = false;
        mShooterTimer.reset();
        mShooterTimer.stop();
    }

    @Override
    public void getToState() {
        switch ((ScollectorState) getState()) {
            case IDLE:
                mCollectorMotor.set(0);
                mRightShooterMotor.set(0);
                mLeftShooterMotor.set(0);
                break;
            case COLLECT:
                collect();
                mLeftShooterMotor.set(0);
                mRightShooterMotor.set(0);
                break;
            case COLLECT_RAMP:
                collect();
                // mLeftShooterMotor.set(1);
                // mRightShooterMotor.set(1);

                break;
            case SPIT:
                mCollectorMotor.set(1);
                mRightPID.setReference(0, ControlType.kVelocity);
                break;
            case SHOOT:
                shoot();
            case COLLECT_AUTO_SHOOT:
                if (!hasNote()) {
                    collect();
                } else {
                    shoot();
                }

                driveMotors();
                break;
            case RAMP_SHOOTER:
                driveMotors();
                mCollectorMotor.set(0);
                break;
            case SEND_NOTE_SHOOT:
                driveMotors();
                mCollectorMotor.set(-1);
                break;
        }
    }

    @Override
    public boolean matchesDesiredState() {
        switch ((ScollectorState) getState()) {
            case IDLE:
                return mCollectorMotor.get() == 0;
            case COLLECT:
                return hasNote();
            case SPIT:
                return mCollectorMotor.get() == -1;
            case SHOOT:
                return !hasNote();
            case COLLECT_RAMP:
                return hasNote();
            case COLLECT_AUTO_SHOOT:
                return mHasShot;
            default:
                return false;
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
        // System.out.println(hasNote() + " :: " + mNoteSensor.getRange());
    }

    public boolean hasNote() {
        return mNoteSensor.getRange() < 300;
    }

    public boolean shooterAtSpeed() {
        double leftSpeed = mLeftShooterMotor.getEncoder().getVelocity();
        double rightSpeed = mRightShooterMotor.getEncoder().getVelocity();
        double diff = Math.abs(leftSpeed - rightSpeed);
        double threshold = 7;

        System.out.println(leftSpeed + " :: " + rightSpeed);

        return leftSpeed >= threshold && rightSpeed >= threshold && diff <= 0.1;
    }

    public void setArmReadyToShoot(boolean armReady) {
        mArmInPosition = armReady;
    }

    private void driveMotors() {
        mRightPID.setReference(7.4, ControlType.kVelocity);
        mLeftPID.setReference(7.4, ControlType.kVelocity);
    }

    private void collect() {
        if (!hasNote()) {
            mCollectorMotor.set(-1);
        } else {
            mCollectorMotor.set(0);
            mHasShot = false;
        }
    }


    private void shoot() {
        final double desiredSpeed = 7.4;

        // double leftDutyCycle = 1; //mLeftShooterPID.calculate(mLeftShooterMotor.getEncoder().getVelocity(), desiredSpeed);
        // double rightDutyCycle = 1; //mRightShooterPID.calculate(mRightShooterMotor.getEncoder().getVelocity(), desiredSpeed);

        // mLeftShooterMotor.set(leftDutyCycle);
        // mRightShooterMotor.set(rightDutyCycle);
        mRightPID.setReference(desiredSpeed, ControlType.kVelocity);
        mLeftPID.setReference(desiredSpeed, ControlType.kVelocity);

        if (shooterAtSpeed()) {
            mShooterTimer.start();
        }

        if (!mArmInPosition)
            return;

        if (mShooterTimer.get() >= 0.1 && mShooterTimer.get() <= 1.5) {

            if (!hasNote()) {
                mHasShot = false;
            }
        } else if (mShooterTimer.get() > 1.5) {
            mShooterTimer.stop();
            mShooterTimer.reset();
            mCollectorMotor.set(0);
        } else {
        }


    }

}
