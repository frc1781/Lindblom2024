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

//EXAMPLE SUBSYSTEM, NOT FOR ACTUAL BOT
public class Scollector extends Subsystem {
    private CANSparkMax mCollectorMotor = new CANSparkMax(ConfigMap.COLLECTOR_MOTOR, CANSparkMax.MotorType.kBrushless);
    private CANSparkMax mRightShooterMotor = new CANSparkMax(ConfigMap.SHOOTER_RIGHT_MOTOR,
            CANSparkMax.MotorType.kBrushless);
    private CANSparkMax mLeftShooterMotor = new CANSparkMax(ConfigMap.SHOOTER_LEFT_MOTOR,
            CANSparkMax.MotorType.kBrushless);

    private GenericEntry mThresholdEntry = ConfigMap.SHUFFLEBOARD_TAB.add("Shooter Threshold", 6).getEntry();

    private ProfiledPIDController mLeftShooterPID = new ProfiledPIDController(2, 0, 0,
            new TrapezoidProfile.Constraints(10, 2));
    private ProfiledPIDController mRightShooterPID = new ProfiledPIDController(2, 0, 0,
            new TrapezoidProfile.Constraints(10, 2));

    private TimeOfFlight mNoteSensor = new TimeOfFlight(ConfigMap.SCOLLECTOR_TOF);

    private boolean mIsReadyToShoot = false;
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
        mShooterTimer.reset();
        mShooterTimer.start();
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

    public void setReadyToShoot(boolean canShoot) {
        mIsReadyToShoot = canShoot;
    }

    private void collect() {

        if (!hasNote()) {
            mCollectorMotor.set(-1);
        } else {
            mCollectorMotor.set(0);
        }
    }

    private void shoot() {
        double threshold = mThresholdEntry.getDouble(7);
        // System.out.println("Left: " + mLeftShooterMotor.getEncoder().getVelocity());
        // System.out.println("Right: " +
        // mRightShooterMotor.getEncoder().getVelocity());
        // System.out.println("Average: "
        // + (mLeftShooterMotor.getEncoder().getVelocity() +
        // mRightShooterMotor.getEncoder().getVelocity()) / 2);

        double leftDutyCycle = 1;// mLeftShooterPID.calculate(mLeftShooterMotor.getEncoder().getVelocity(),
                                 // threshold);
        double rightDutyCycle = 1; // mRightShooterPID.calculate(mRightShooterMotor.getEncoder().getVelocity(),
                                   // threshold);

        mLeftShooterMotor.set(leftDutyCycle);
        mRightShooterMotor.set(rightDutyCycle);

        if (shooterAtSpeed() && mIsReadyToShoot) {
            mCollectorMotor.set(-1);
        } else {
            // System.out.println(mLeftShooterMotor.getEncoder().getVelocity());
            mCollectorMotor.set(0);
        }
    }

    private boolean shooterAtSpeed() {
        double leftSpeed = mLeftShooterMotor.getEncoder().getVelocity();
        double rightSpeed = mRightShooterMotor.getEncoder().getVelocity();

        double threshold = mThresholdEntry.getDouble(7);

        if (leftSpeed >= threshold && rightSpeed >= threshold) {
            if (mShooterTimer.get() > 0.5) {
                mShooterTimer.reset();
                return true;
            }
        } else {
            mShooterTimer.reset();
        }

        return false;
    }
}
