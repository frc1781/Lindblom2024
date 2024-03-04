package tech.team1781.subsystems;
import tech.team1781.utils.NetworkLogger;
import com.revrobotics.CANSparkMax;
import tech.team1781.ConfigMap;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

public class Climber extends Subsystem {
    private CANSparkMax mLeftClimberMotor = new CANSparkMax(ConfigMap.LEFT_CLIMBER_MOTOR,
            CANSparkMax.MotorType.kBrushless);
    private CANSparkMax mRightClimberMotor = new CANSparkMax(ConfigMap.RIGHT_CLIMBER_MOTOR,
            CANSparkMax.MotorType.kBrushless);
    private SparkLimitSwitch mLeftReverseLimitSwitch = mLeftClimberMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    private SparkLimitSwitch mRightReverseLimitSwitch = mRightClimberMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    private SparkLimitSwitch mLeftForwardLimitSwitch = mLeftClimberMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    private SparkLimitSwitch mRightForwardLimitSwitch = mRightClimberMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    private PIDController mRightClimberPID = new PIDController(0.1, 0, 0);
    private RelativeEncoder mLeftClimberEncoder = mLeftClimberMotor.getEncoder();
    private RelativeEncoder mRightClimberEncoder = mRightClimberMotor.getEncoder();

    public Climber() {
        super("Climber", ClimberState.IDLE);
        mLeftClimberMotor.setInverted(false);
        mRightClimberMotor.setInverted(true);
        mLeftClimberMotor.setIdleMode(IdleMode.kBrake);
        mRightClimberMotor.setIdleMode(IdleMode.kBrake);
        mLeftClimberMotor.setSmartCurrentLimit(40);
        mRightClimberMotor.setSmartCurrentLimit(40);
        mLeftReverseLimitSwitch = mLeftClimberMotor.getReverseLimitSwitch(Type.kNormallyOpen);
        mRightReverseLimitSwitch = mRightClimberMotor.getReverseLimitSwitch(Type.kNormallyOpen);
        mLeftForwardLimitSwitch = mLeftClimberMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        mRightForwardLimitSwitch = mRightClimberMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        mLeftClimberMotor.burnFlash();
        mRightClimberMotor.burnFlash();
    }

    public enum ClimberState implements Subsystem.SubsystemState {
        IDLE, EXTEND, RETRACT
    }

    @Override
    public void genericPeriodic() {
        if(mLeftReverseLimitSwitch.isPressed()) {
            mLeftClimberEncoder.setPosition(0);
        }

        if(mRightReverseLimitSwitch.isPressed()) {
            mRightClimberEncoder.setPosition(0);
        }
    }

    @Override
    public void init() {
    }

    @Override
    public void getToState() {
   
    }

    @Override
    public boolean matchesDesiredState() {
        return false;
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }

    public void manualClimb(double dutyCycle) {
        if (Math.abs(dutyCycle) <= 0.1) {
            dutyCycle = 0;
        }

        double leftDutyCycle; 
        double rightDutyCycle; 

        if (dutyCycle < 0) {
            leftDutyCycle = dutyCycle;
            rightDutyCycle = dutyCycle;
        } else {
            leftDutyCycle = dutyCycle * 0.7;
            rightDutyCycle = dutyCycle * 0.7;
        }

        rightDutyCycle = dutyCycle + mRightClimberPID.calculate(
            mRightClimberMotor.getEncoder().getPosition(), 
            mLeftClimberMotor.getEncoder().getPosition()
        );

        if (Math.abs(dutyCycle) > 0.1) {
           System.out.printf("lc: %.2f  rc: %.2f re: %.2f  le:%.2f\n", 
             leftDutyCycle,
             rightDutyCycle,
             mLeftClimberMotor.getEncoder().getPosition(),
             mRightClimberMotor.getEncoder().getPosition()
           );
        }
        mLeftClimberMotor.set(leftDutyCycle);
        mRightClimberMotor.set(rightDutyCycle);
    }
}

