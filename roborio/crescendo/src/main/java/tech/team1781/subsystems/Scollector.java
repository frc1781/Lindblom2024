package tech.team1781.subsystems;

import java.util.ArrayList;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
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

    private GenericEntry mThresholdEntry = ConfigMap.SHUFFLEBOARD_TAB.add("Shooter Threshold", 6).getEntry();


    private final EVector SHOOTER_PID = EVector.newVector(2.0, 0.0, 0.0);
    private final TrapezoidProfile.Constraints SHOOTER_CONSTRAINTS = new TrapezoidProfile.Constraints(10, 5);
    private ProfiledPIDController mLeftShooterPID = new ProfiledPIDController(SHOOTER_PID.x, SHOOTER_PID.y,
            SHOOTER_PID.z, SHOOTER_CONSTRAINTS); 
    private ProfiledPIDController mRightShooterPID = new ProfiledPIDController(SHOOTER_PID.x, SHOOTER_PID.y,
            SHOOTER_PID.z, SHOOTER_CONSTRAINTS);

    private TimeOfFlight mNoteSensor = new TimeOfFlight(ConfigMap.SCOLLECTOR_TOF);

    private boolean mIsReadyToShoot = false;
    private boolean mArmInPosition = false;
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
    }

    public enum ScollectorState implements SubsystemState {
        IDLE, COLLECT, SPIT, SHOOT, COLLECT_RAMP
    }

    @Override
    public void genericPeriodic() {
    }

    @Override
    public void init() {
        mIsReadyToShoot = false;
        mArmInPosition = false;
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
                mLeftShooterMotor.set(1);
                mRightShooterMotor.set(1);
                break;
            case SPIT:
                mCollectorMotor.set(1);
                break;
            case SHOOT:
                shoot();
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

    public void setArmReadyToShoot(boolean armReady) {
        mArmInPosition = armReady;
    }

    private void collect() {

        if (!hasNote()) {
            mCollectorMotor.set(-1);
        } else {
            mCollectorMotor.set(0);
        }
    }

    private boolean isSending = false;

    private void shoot() {
        final double desiredSpeed = 7;
        
        double leftDutyCycle = mLeftShooterPID.calculate(mLeftShooterMotor.getEncoder().getVelocity(), desiredSpeed);
        double rightDutyCycle = mRightShooterPID.calculate(mRightShooterMotor.getEncoder().getVelocity(), desiredSpeed);

        mLeftShooterMotor.set(leftDutyCycle);
        mRightShooterMotor.set(rightDutyCycle);

        if (shooterAtSpeed()) {
            mShooterTimer.start();
        }

        // if(mArmInPosition)
        // return;
        System.out.println(mRightShooterMotor.getEncoder().getPosition() + "," +
                mLeftShooterMotor.getEncoder().getPosition() + "," +
                mRightShooterMotor.getEncoder().getVelocity() + "," +
                mLeftShooterMotor.getEncoder().getVelocity() + "," +
                (isSending));

        if (mShooterTimer.get() >= 0.5 && mShooterTimer.get() <= 1.5) {
            mCollectorMotor.set(-1);
            isSending = true;
        } else if (mShooterTimer.get() > 1.5) {
            mShooterTimer.stop();
            mShooterTimer.reset();
            mCollectorMotor.set(0);
            isSending = false;
        } else {
            isSending = false;
        }

    }

    private boolean shooterAtSpeed() {
        double leftSpeed = mLeftShooterMotor.getEncoder().getVelocity();
        double rightSpeed = mRightShooterMotor.getEncoder().getVelocity();

        double threshold = mThresholdEntry.getDouble(7);

        return leftSpeed >= threshold && rightSpeed >= threshold;
    }
}
