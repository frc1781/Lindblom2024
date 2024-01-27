package tech.team1781.subsystems;

// import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import tech.team1781.ConfigMap;

//EXAMPLE SUBSYSTEM, NOT FOR ACTUAL BOT
public class Scollector extends Subsystem {
    private CANSparkMax mCollectorMotor = new CANSparkMax(ConfigMap.COLLECTOR_MOTOR, CANSparkMax.MotorType.kBrushless);
    private CANSparkMax mRightShooterMotor = new CANSparkMax(ConfigMap.SHOOTER_RIGHT_MOTOR,
            CANSparkMax.MotorType.kBrushless);
    private CANSparkMax mLeftShooterMotor = new CANSparkMax(ConfigMap.SHOOTER_LEFT_MOTOR,
            CANSparkMax.MotorType.kBrushless);

    private ProfiledPIDController mLeftShooterPID = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(1000, 100));
    private ProfiledPIDController mRightShooterPID = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(1000, 100));
    // private TimeOfFlight mTimeOfFlight = new
    // TimeOfFlight(ConfigMap.COLLECTOR_TOF);
    // private CANSparkMax mTopShooterMotor = new
    // CANSparkMax(ConfigMap.TOP_SHOOTER_MOTOR,
    // CANSparkMax.MotorType.kBrushless);
    // private CANSparkMax mShooterMotor = new CANSparkMax(ConfigMap.SHOOTER_MOTOR,
    // CANSparkMax.MotorType.kBrushless);

    public Scollector() {
        super("Scollector", ScollectorState.IDLE);
        mCollectorMotor.setIdleMode(IdleMode.kBrake);
        mLeftShooterMotor.setIdleMode(IdleMode.kCoast);
        mRightShooterMotor.setIdleMode(IdleMode.kCoast);

        final double conversionFactor = 0.100203 * 1/60;

        mLeftShooterMotor.getEncoder().setPositionConversionFactor(conversionFactor);
        mLeftShooterMotor.getEncoder().setVelocityConversionFactor(conversionFactor);
        mRightShooterMotor.getEncoder().setPositionConversionFactor(conversionFactor);
        mRightShooterMotor.getEncoder().setVelocityConversionFactor(conversionFactor);
    }

    public enum ScollectorState implements SubsystemState {
        IDLE, COLLECT, SPIT, SHOOT
    }

    @Override
    public void genericPeriodic() {
    }

    @Override
    public void init() {
    }

    // public boolean hasNote() {
    // if (mTimeOfFlight.getRange() < 10) {
    // return true;
    // } else {
    // return false;
    // }
    // }

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
                return false;
            case SPIT:
                return mCollectorMotor.get() == -1;
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

    private void collect() {
        mCollectorMotor.set(-1);
    }

    private void shoot() {
        final double threshold = 8;
        System.out.println("Left: " + mLeftShooterMotor.getEncoder().getVelocity());
        System.out.println("Right: " + mRightShooterMotor.getEncoder().getVelocity());
        System.out.println("Average: " + (mLeftShooterMotor.getEncoder().getVelocity() + mRightShooterMotor.getEncoder().getVelocity()) / 2);

        double leftDutyCycle = mLeftShooterPID.calculate(mLeftShooterMotor.getEncoder().getVelocity(), threshold);
        double rightDutyCycle = mRightShooterPID.calculate(mRightShooterMotor.getEncoder().getVelocity(), threshold);

        mLeftShooterMotor.set(leftDutyCycle);
        mRightShooterMotor.set(rightDutyCycle);

        if (mLeftShooterMotor.getEncoder().getVelocity() > threshold && mRightShooterMotor.getEncoder().getVelocity() > threshold) {
            mCollectorMotor.set(-1);
        }
        else {
            // System.out.println(mLeftShooterMotor.getEncoder().getVelocity());
            mCollectorMotor.set(0);
        }
    }
}
