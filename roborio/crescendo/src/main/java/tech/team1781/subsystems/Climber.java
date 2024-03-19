package tech.team1781.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkLimitSwitch.Type;
import tech.team1781.ConfigMap;
import edu.wpi.first.math.controller.PIDController;

public class Climber extends Subsystem {
    private HookState mRightHookState = HookState.DOWN;
    private HookState mLeftHookState = HookState.DOWN;
    // private TrapState mTrapState = TrapState.IN;

    private CANSparkMax mLeftClimberMotor = new CANSparkMax(
        ConfigMap.LEFT_CLIMBER_MOTOR,
        CANSparkMax.MotorType.kBrushless);
    private CANSparkMax mRightClimberMotor = new CANSparkMax(
        ConfigMap.RIGHT_CLIMBER_MOTOR,
        CANSparkMax.MotorType.kBrushless);
    private CANSparkMax mTrapHookMotor = new CANSparkMax(
        ConfigMap.TRAP_HOOK_MOTOR, 
        CANSparkMax.MotorType.kBrushless);

    private SparkLimitSwitch mLeftReverseLimitSwitch = mLeftClimberMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    private SparkLimitSwitch mRightReverseLimitSwitch = mRightClimberMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    private SparkLimitSwitch mLeftForwardLimitSwitch = mLeftClimberMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    private SparkLimitSwitch mRightForwardLimitSwitch = mRightClimberMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    private PIDController mRightClimberPID = new PIDController(0.1, 0, 0);
    private RelativeEncoder mLeftClimberEncoder = mLeftClimberMotor.getEncoder();
    private RelativeEncoder mRightClimberEncoder = mRightClimberMotor.getEncoder();
    private RelativeEncoder mTrapHookEncoder = mTrapHookMotor.getEncoder();

    public Climber() {
        super("Climber", ClimberState.IDLE);
        mLeftClimberMotor.setInverted(false);
        mRightClimberMotor.setInverted(true);
        mTrapHookMotor.setInverted(true); //SUBJECT TO CHANGE
        mLeftClimberMotor.setIdleMode(IdleMode.kBrake);
        mRightClimberMotor.setIdleMode(IdleMode.kBrake);
        mTrapHookMotor.setIdleMode(IdleMode.kBrake);
        mLeftClimberMotor.setSmartCurrentLimit(40);
        mRightClimberMotor.setSmartCurrentLimit(40);
        mTrapHookMotor.setSmartCurrentLimit(42);
        mLeftReverseLimitSwitch = mLeftClimberMotor.getReverseLimitSwitch(Type.kNormallyOpen);
        mRightReverseLimitSwitch = mRightClimberMotor.getReverseLimitSwitch(Type.kNormallyOpen);
        mLeftForwardLimitSwitch = mLeftClimberMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        mRightForwardLimitSwitch = mRightClimberMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        mRightClimberEncoder.setPosition(0);
        mLeftClimberEncoder.setPosition(0);
        mTrapHookEncoder.setPosition(0);
        mLeftClimberMotor.burnFlash();
        mRightClimberMotor.burnFlash();
        mTrapHookMotor.burnFlash();
    }

    public enum ClimberState implements Subsystem.SubsystemState {
        IDLE, EXTEND, RETRACT
    }

    public enum HookState implements Subsystem.SubsystemState {
        UP, DOWN
    }

    // public enum TrapState implements Subsystem.SubsystemState {
    //     IN, OUT
    // }

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
        mRightClimberPID.reset();
        // mTrapState = TrapState.IN;
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

    public void toggleTrap() {
        // setTrap(mTrapState == TrapState.IN ? TrapState.OUT : TrapState.IN);
        System.out.println("????????????????????????????????????");
    }

    public void setHooks(HookState h) {
        if (mLeftHookState == h) {
            return; 
        }
        mLeftHookState = h;
        mRightHookState = h;
        System.out.println(h == HookState.UP ? "Hooks up" : "Hooks Down");
    }

    // public void setTrap(TrapState t) {
    //     if (mTrapState != t) {
    //       mTrapState = t;
    //       System.out.println("Trap set to " + mTrapState);
    //     }
    // }

    public void manualClimb(double dutyCycle) {
        if (Math.abs(dutyCycle) <= 0.1) {
            dutyCycle = 0;
            mRightClimberPID.reset();
        }

        double leftDutyCycle; 
        double rightDutyCycle; 

        if (dutyCycle < 0) {
            leftDutyCycle = dutyCycle  * 0.9;
            rightDutyCycle = dutyCycle + mRightClimberPID.calculate(
                mRightClimberMotor.getEncoder().getPosition(),
                mLeftClimberMotor.getEncoder().getPosition()
                );
        } else {
            leftDutyCycle = dutyCycle * 0.7;
            rightDutyCycle = dutyCycle * 0.7;
        }


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


    public void twoThumbClimb(double dutyCycleLeft, double dutyCycleRight) {
        if (Math.abs(dutyCycleLeft) <= 0.1) {
            dutyCycleLeft = 0;
        } 
        if(Math.abs(dutyCycleRight) <= 0.1) {
            dutyCycleRight = 0;
        }

        dutyCycleLeft = dutyCycleLeft < 0 ? dutyCycleLeft: dutyCycleLeft * 0.7; 
        dutyCycleRight = dutyCycleRight < 0 ? dutyCycleRight: dutyCycleRight * 0.7; 

        mLeftClimberMotor.set(dutyCycleLeft);
        mRightClimberMotor.set(dutyCycleRight);
    }

    public void pullTrapHooks() {

    }
}
