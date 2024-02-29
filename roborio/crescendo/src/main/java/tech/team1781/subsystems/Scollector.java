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
import tech.team1781.ShuffleboardStyle;
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


    private final EVector SHOOTER_PID = EVector.newVector(0.1, 0.0, 0.0);
    private final TrapezoidProfile.Constraints SHOOTER_CONSTRAINTS = new TrapezoidProfile.Constraints(9.0, 10);
    private ProfiledPIDController mBottomShooterPID = new ProfiledPIDController(SHOOTER_PID.x, SHOOTER_PID.y,
            SHOOTER_PID.z, SHOOTER_CONSTRAINTS);
    private ProfiledPIDController mTopShooterPID = new ProfiledPIDController(SHOOTER_PID.x, SHOOTER_PID.y,
            SHOOTER_PID.z, SHOOTER_CONSTRAINTS);

    private TimeOfFlight mTopTof = new TimeOfFlight(ConfigMap.TOP_SCOLLECTOR_TOF);
    private TimeOfFlight mBottomTof = new TimeOfFlight(ConfigMap.BOTTOM_SCOLLECTOR_TOF);

    private boolean mArmInPosition = false;
    private Timer mShooterTimer = new Timer();


    private GenericEntry mTopShooterVelocity = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Top Velocity", 0, ShuffleboardStyle.TOP_SHOOTER); 
    private GenericEntry mBottomShooterVelocity = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Bottom Velocity", 0, ShuffleboardStyle.BOTTOM_SHOOTER);
    private GenericEntry mReadyToShootEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Ready To Shoot", true, ShuffleboardStyle.READY_TO_SHOOT);
    private GenericEntry mHasNoteEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Has Note", false, ShuffleboardStyle.HAS_NOTE);

    public Scollector() {
        super("Scollector", ScollectorState.IDLE);
        mCollectorMotor.setIdleMode(IdleMode.kBrake);
        mBottomShooterMotor.setIdleMode(IdleMode.kCoast);
        mTopShooterMotor.setIdleMode(IdleMode.kCoast);
        mTopShooterMotor.setInverted(false);
        mBottomShooterMotor.setInverted(true);

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

        final EVector SHOOTER_PID = EVector.newVector(0.5,0,0.1);
        final double SHOOTER_FF = 1/9.8;

        mTopPID.setP(SHOOTER_PID.x);
        mTopPID.setI(SHOOTER_PID.y);
        mTopPID.setD(SHOOTER_PID.z);
        mTopPID.setFF(SHOOTER_FF);

        mBottomPID.setP(SHOOTER_PID.x);
        mBottomPID.setI(SHOOTER_PID.y);
        mBottomPID.setD(SHOOTER_PID.z);
        mBottomPID.setFF(SHOOTER_FF);
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
        mReadyToShootEntry.setBoolean(shooterAtSpeed());
        mHasNoteEntry.setBoolean(hasNote());
    }

    @Override
    public void init() {
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
                } else if (mArmInPosition && (noteCloseToShooter() || hasNote()) && shooterAtSpeed()) {
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
        double range = mBottomTof.getRange();
        if(!mBottomTof.isRangeValid() && range == 0.0) {
            return false;
        }      

        return range <= 400;
    }

    public boolean noteCloseToShooter() {
        if(!mTopTof.isRangeValid() && mTopTof.getRange() == 0.0) {
            return false;
        }
        return mTopTof.getRange() <= 400;
    }

    public boolean shooterAtSpeed() {
        double leftSpeed = mBottomShooterMotor.getEncoder().getVelocity();
        double rightSpeed = mTopShooterMotor.getEncoder().getVelocity();
        double leftDiff = Math.abs(leftSpeed - ConfigMap.MAX_SHOOTER_SPEED);
        double rightDiff = Math.abs(rightSpeed - ConfigMap.MAX_SHOOTER_SPEED);
        final double TOLERANCE = 0.1;

        return leftSpeed >= ConfigMap.MAX_SHOOTER_SPEED && rightSpeed >= ConfigMap.MAX_SHOOTER_SPEED;
        
    }

    public void setArmReadyToShoot(boolean armReady) {
        mArmInPosition = armReady;
    }

    private void driveMotors() {
        double setpoint = ConfigMap.MAX_SHOOTER_SPEED;
        mTopPID.setReference(setpoint, ControlType.kVelocity);
        mBottomPID.setReference(setpoint, ControlType.kVelocity);
    }

    private void collect() {
        if (!hasNote() && !noteCloseToShooter()) {
            mCollectorMotor.set(-1);
        } else if(noteCloseToShooter()){
            mCollectorMotor.set(0.25);
        } else if(hasNote()){
            mCollectorMotor.set(0);
        }
    }

    private void shoot() {
        driveMotors();
        mCollectorMotor.set(-1);
    }
    
}
