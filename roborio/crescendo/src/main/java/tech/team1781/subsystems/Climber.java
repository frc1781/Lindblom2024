package tech.team1781.subsystems;
import tech.team1781.utils.NetworkLogger;
import com.revrobotics.CANSparkMax;
import tech.team1781.ConfigMap;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public class Climber extends Subsystem {
    HookState mRightHookState = HookState.DOWN;
    HookState mLeftHookState = HookState.DOWN;
    TrapState mTrapState = TrapState.IN;

    private CANSparkMax mLeftClimberMotor = new CANSparkMax(ConfigMap.LEFT_CLIMBER_MOTOR,
            CANSparkMax.MotorType.kBrushless);
    private CANSparkMax mRightClimberMotor = new CANSparkMax(ConfigMap.RIGHT_CLIMBER_MOTOR,
            CANSparkMax.MotorType.kBrushless);
     private static DoubleSolenoid mLeftHook = new DoubleSolenoid(
        ConfigMap.FIRST_PCM_ID,
        PneumaticsModuleType.CTREPCM, 
        ConfigMap.LEFT_HOOK_OPEN, 
        ConfigMap.LEFT_HOOK_CLOSE);
    private static DoubleSolenoid mRightHook = new DoubleSolenoid(
        ConfigMap.FIRST_PCM_ID,
        PneumaticsModuleType.CTREPCM, 
        ConfigMap.RIGHT_HOOK_OPEN, 
        ConfigMap.RIGHT_HOOK_CLOSE);
    private static DoubleSolenoid mTrap = new DoubleSolenoid(
        ConfigMap.FIRST_PCM_ID,
        PneumaticsModuleType.CTREPCM, 
        ConfigMap.TRAP_IN, 
        ConfigMap.TRAP_OUT); 

    public Climber() {
        super("Climber", ClimberState.IDLE);
        mLeftClimberMotor.setInverted(true);
        mRightClimberMotor.setInverted(false);
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
    }

    @Override
    public void init() {
    }

    @Override
    public void getToState() {
        double leftDC = 0;
        double rightDC = 0;
        mLeftHook.set(mLeftHookState == HookState.UP ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
        mRightHook.set(mRightHookState == HookState.UP ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
        mTrap.set(mTrapState == TrapState.OUT ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
        // switch ((ClimberState) getState()) {
        //     case IDLE:
        //         leftDC = 0;
        //         rightDC = 0;
        //         break;
        //     case EXTEND:
        //         leftDC = 0.3;
        //         rightDC = 0.3;
        //         break;
        //     case RETRACT:
        //         leftDC = -0.3;
        //         rightDC = -0.3;
        //         break;
        // }

        // // System.out.printf("left dc: %.2f right dc: %.2f\n",
        // //     leftDC,
        // //     rightDC
        // // );       
        // mLeftClimberMotor.set(leftDC);
        // mRightClimberMotor.set(rightDC);     
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
        if(Math.abs(dutyCycle) <= 0.1) {
            dutyCycle = 0;
        }

        mLeftClimberMotor.set(dutyCycle);
        mRightClimberMotor.set(dutyCycle);
    }

}

