package tech.team1781.subsystems;

import java.util.ArrayList;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkFlex;
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
    private CANSparkFlex mTopShooterMotor = new CANSparkFlex(ConfigMap.SHOOTER_TOP_MOTOR,
            CANSparkFlex.MotorType.kBrushless);
    private CANSparkFlex mBottomShooterMotor = new CANSparkFlex(ConfigMap.SHOOTER_BOTTOM_MOTOR,
            CANSparkFlex.MotorType.kBrushless);

    private final SparkPIDController mTopPID;
    private final SparkPIDController mBottomPID;

    private GenericEntry mThresholdEntry = ConfigMap.SHUFFLEBOARD_TAB.add("Shooter Threshold", 6).getEntry();

    private final EVector SHOOTER_PID = EVector.newVector(0.1, 0.0, 0.0);
    private final TrapezoidProfile.Constraints SHOOTER_CONSTRAINTS = new TrapezoidProfile.Constraints(9.0, 10);
    private ProfiledPIDController mBottomShooterPID = new ProfiledPIDController(SHOOTER_PID.x, SHOOTER_PID.y,
            SHOOTER_PID.z, SHOOTER_CONSTRAINTS);
    private ProfiledPIDController mTopShooterPID = new ProfiledPIDController(SHOOTER_PID.x, SHOOTER_PID.y,
            SHOOTER_PID.z, SHOOTER_CONSTRAINTS);

    private TimeOfFlight mNoteSensor = new TimeOfFlight(ConfigMap.SCOLLECTOR_TOF);

    private boolean mIsReadyToShoot = false;
    private boolean mArmInPosition = false;
    private boolean mHasShot = false;
    private Timer mShooterTimer = new Timer();

    private GenericEntry mTopShooterVelocity = ConfigMap.SHUFFLEBOARD_TAB.add("Top Velocity", 0).getEntry();
    private GenericEntry mBottomShooterVelocity = ConfigMap.SHUFFLEBOARD_TAB.add("Bottom Velocity", 0).getEntry();

    public Scollector() {
        super("Scollector", ScollectorState.IDLE);
        mCollectorMotor.setIdleMode(IdleMode.kBrake);
        mBottomShooterMotor.setIdleMode(IdleMode.kCoast);
        mTopShooterMotor.setIdleMode(IdleMode.kCoast);
        mTopShooterMotor.setInverted(false);
        mBottomShooterMotor.setInverted(false);

        final double conversionFactor = 0.100203 * 1 / 60;

        mBottomShooterMotor.getEncoder().setPositionConversionFactor(conversionFactor);
        mBottomShooterMotor.getEncoder().setVelocityConversionFactor(conversionFactor);
        mTopShooterMotor.getEncoder().setPositionConversionFactor(conversionFactor);
        mTopShooterMotor.getEncoder().setVelocityConversionFactor(conversionFactor);
        mBottomShooterMotor.setSmartCurrentLimit(30);
        mTopShooterMotor.setSmartCurrentLimit(30);
        mTopPID = mTopShooterMotor.getPIDController();
        mBottomPID = mBottomShooterMotor.getPIDController();
        mTopPID.setFeedbackDevice(mTopShooterMotor.getEncoder());
        mBottomPID.setFeedbackDevice(mBottomShooterMotor.getEncoder());
        mTopPID.setP(0.5);
        mTopPID.setI(0);
        mTopPID.setD(0.01);
        mTopPID.setFF(1.0 / 9.8);

        mBottomPID.setP(0.5);
        mBottomPID.setI(0);
        mBottomPID.setD(0.01);
        mBottomPID.setFF(1.0 / 9.8);
        mBottomShooterMotor.burnFlash();
        mTopShooterMotor.burnFlash();
        mCollectorMotor.burnFlash();
        System.out.println("top motor faults: " + mTopShooterMotor.getFaults());
        System.out.println("top motor faults: " + mBottomShooterMotor.getFaults());
    }

    public enum ScollectorState implements SubsystemState {
        IDLE, COLLECT, SPIT, SHOOT, COLLECT_RAMP, COLLECT_AUTO_SHOOT, RAMP_SHOOTER
    }

    @Override
    public void genericPeriodic() {
        mTopShooterVelocity.setDouble(mTopShooterMotor.getEncoder().getVelocity());
        mBottomShooterVelocity.setDouble(mBottomShooterMotor.getEncoder().getVelocity());
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
                mTopShooterMotor.set(0);
                mBottomShooterMotor.set(0);
                break;
            case COLLECT:
                collect();
                mBottomShooterMotor.set(0);
                mTopShooterMotor.set(0);
                break;
            case COLLECT_RAMP:
                collect();
                driveMotors();
                break;
            case SPIT:
                mCollectorMotor.set(1);
                mTopPID.setReference(0, ControlType.kVelocity);
                break;
            case SHOOT:
                shoot();
                break;
            case COLLECT_AUTO_SHOOT:
                if (!hasNote()) {
                    collect();
                } else if (mArmInPosition) {
                    shoot();
                } else {
                    mCollectorMotor.set(0);
                }

                driveMotors();
                break;
            case RAMP_SHOOTER:
                driveMotors();
                mCollectorMotor.set(0);
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
                return false;
            default:
                return false;
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }

    public boolean hasNote() {
        // return false;
        return mNoteSensor.getRange() < 300;
    }

    public boolean shooterAtSpeed() {
        double leftSpeed = mBottomShooterMotor.getEncoder().getVelocity();
        double rightSpeed = mTopShooterMotor.getEncoder().getVelocity();
        double diff = Math.abs(leftSpeed - rightSpeed);
        double threshold = 7;
        return leftSpeed >= threshold && rightSpeed >= threshold && diff <= 0.1;
    }

    public void setArmReadyToShoot(boolean armReady) {
        mArmInPosition = armReady;
    }

    private void driveMotors() {
        // mTopShooterMotor.set(1);
        // mBottomShooterMotor.set(1);
        double setpoint = 8;
        mTopPID.setReference(setpoint, ControlType.kVelocity);
        mBottomPID.setReference(setpoint, ControlType.kVelocity);
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
        // final double desiredSpeed = 9;

        //mTopPID.setReference(desiredSpeed, ControlType.kVelocity);
        //mBottomPID.setReference(desiredSpeed, ControlType.kVelocity);

        driveMotors();
        // if (shooterAtSpeed()) {
        //     mShooterTimer.start();
        // }

        // if (!mArmInPosition)
        //     return;

        // if (mShooterTimer.get() >= 0.1 && mShooterTimer.get() <= 1.5) {
            mCollectorMotor.set(-1);
    //         if (!hasNote()) {
    //             mHasShot = false;
    //         }
    //     } else if (mShooterTimer.get() > 1.5) {
    //         mShooterTimer.stop();
    //         mShooterTimer.reset();
    //         mCollectorMotor.set(0);
    //     } else {
    //     }
    // }
    }

}
