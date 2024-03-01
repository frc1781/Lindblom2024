package tech.team1781.subsystems;

import tech.team1781.utils.NetworkLogger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkLimitSwitch.Type;

import tech.team1781.ConfigMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class Climber extends Subsystem {
    private HookState mRightHookState = HookState.DOWN;
    private HookState mLeftHookState = HookState.DOWN;
    private TrapState mTrapState = TrapState.IN;

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
    private static DoubleSolenoid mTrap = new DoubleSolenoid(
            ConfigMap.FIRST_PCM_ID,
            PneumaticsModuleType.REVPH,
            ConfigMap.TRAP_IN,
            ConfigMap.TRAP_OUT);

    private SparkLimitSwitch mLeftLimitSwitch = mLeftClimberMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    private SparkLimitSwitch mRightLimitSwitch = mRightClimberMotor.getForwardLimitSwitch(Type.kNormallyOpen);

    private PIDController mRightClimberPID = new PIDController(0.1, 0, 0);


    public Climber() {
        super("Climber", ClimberState.IDLE);
        mLeftClimberMotor.setInverted(true);
        mRightClimberMotor.setInverted(false);

        mLeftLimitSwitch = mLeftClimberMotor.getForwardLimitSwitch(Type.kNormallyOpen);
        mRightLimitSwitch = mRightClimberMotor.getForwardLimitSwitch(Type.kNormallyOpen);
    }

    public enum ClimberState implements Subsystem.SubsystemState {
        IDLE, EXTEND, RETRACT
    }

    public enum HookState implements Subsystem.SubsystemState {
        UP, DOWN
    }

    public enum TrapState implements Subsystem.SubsystemState {
        IN, OUT
    }

    @Override
    public void genericPeriodic() {
        if(mLeftLimitSwitch.isPressed() && mRightLimitSwitch.isPressed()) {
            mLeftHookState = HookState.UP;
            mRightHookState = HookState.UP;
        } else {
            mLeftHookState = HookState.DOWN;
            mRightHookState = HookState.DOWN;
        }
    }

    @Override
    public void init() {
        mRightClimberPID.reset();
    }

    @Override
    public void getToState() {
        double leftDC = 0;
        double rightDC = 0;
        mLeftHook.set(mLeftHookState == HookState.UP ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
        mRightHook.set(mRightHookState == HookState.UP ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
        mTrap.set(mTrapState == TrapState.OUT ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
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

    public void setHooks(HookState h) {
        mLeftHookState = h;
        mRightHookState = h;
    }

    public void setTrap(TrapState t) {
        mTrapState = t;
    }

    public void manualClimb(double dutyCycle) {
        if (Math.abs(dutyCycle) <= 0.1) {
            dutyCycle = 0;
        }

        mLeftClimberMotor.set(dutyCycle);

        double rightDutyCycle = mRightClimberPID.calculate(mLeftClimberMotor.getEncoder().getPosition(), mRightClimberMotor.getEncoder().getPosition());
        mRightClimberMotor.set(rightDutyCycle);
        
    }

}
