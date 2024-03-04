package tech.team1781.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkLimitSwitch.Type;
import tech.team1781.ConfigMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Climber extends Subsystem {
    private HookState mRightHookState = HookState.DOWN;
    private HookState mLeftHookState = HookState.DOWN;
    // private TrapState mTrapState = TrapState.IN;

    private CANSparkMax mLeftClimberMotor = new CANSparkMax(ConfigMap.LEFT_CLIMBER_MOTOR,
            CANSparkMax.MotorType.kBrushless);
    private CANSparkMax mRightClimberMotor = new CANSparkMax(ConfigMap.RIGHT_CLIMBER_MOTOR,
            CANSparkMax.MotorType.kBrushless);
    private static DoubleSolenoid mLeftHook = new DoubleSolenoid(
            ConfigMap.FIRST_PCM_ID,
            PneumaticsModuleType.REVPH,
            ConfigMap.LEFT_HOOK_OPEN,
            ConfigMap.LEFT_HOOK_CLOSE);
    private static DoubleSolenoid mRightHook = new DoubleSolenoid(
            ConfigMap.FIRST_PCM_ID,
            PneumaticsModuleType.REVPH,
            ConfigMap.RIGHT_HOOK_OPEN,
            ConfigMap.RIGHT_HOOK_CLOSE);
    // private static DoubleSolenoid mTrap = new DoubleSolenoid(
    //         ConfigMap.FIRST_PCM_ID,
    //         PneumaticsModuleType.REVPH,
    //         ConfigMap.TRAP_IN,
    //         ConfigMap.TRAP_OUT);

    private SparkLimitSwitch mLeftReverseLimitSwitch = mLeftClimberMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    private SparkLimitSwitch mRightReverseLimitSwitch = mRightClimberMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    private SparkLimitSwitch mLeftForwardLimitSwitch = mLeftClimberMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    private SparkLimitSwitch mRightForwardLimitSwitch = mRightClimberMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    private PIDController mRightClimberPID = new PIDController(0.2, 0, 0);

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

    public enum HookState implements Subsystem.SubsystemState {
        UP, DOWN
    }

    // public enum TrapState implements Subsystem.SubsystemState {
    //     IN, OUT
    // }

    @Override
    public void genericPeriodic() {
        if(mLeftForwardLimitSwitch.isPressed() && mRightForwardLimitSwitch.isPressed() && mLeftHookState == HookState.DOWN) {
            setHooks(HookState.UP);
        } 
        
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
        mLeftHook.set(mLeftHookState == HookState.UP ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
        mRightHook.set(mRightHookState == HookState.UP ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
        // mTrap.set(mTrapState == TrapState.OUT ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
        double rightClimberEncoderPos = mRightClimberEncoder.getPosition();
        double leftClimberEncoderPos = mLeftClimberEncoder.getPosition();
        double rightClimberDc = mRightClimberPID.calculate(rightClimberEncoderPos, leftClimberEncoderPos);
        mRightClimberMotor.set(rightClimberDc);
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
        }

        if (dutyCycle < 0)  {
          setHooks(HookState.DOWN);
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
        
        
    }

}
