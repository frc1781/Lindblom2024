package tech.team1781.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkLimitSwitch.Type;
import tech.team1781.ConfigMap;
import tech.team1781.utils.NetworkLogger;
import edu.wpi.first.math.controller.PIDController;

public class Climber extends Subsystem {
    private boolean mEngageTrapHook = false;

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
    private PIDController mTrapHookPID = new PIDController(0.1, 0, 0);
    private RelativeEncoder mLeftClimberEncoder = mLeftClimberMotor.getEncoder();
    private RelativeEncoder mRightClimberEncoder = mRightClimberMotor.getEncoder();
    private RelativeEncoder mTrapHookEncoder = mTrapHookMotor.getEncoder();


    
    public Climber() {
        super("Climber", ClimberState.IDLE);
        mLeftClimberMotor.setInverted(false);
        mRightClimberMotor.setInverted(true);
        mTrapHookMotor.setInverted(true); //temporary, we don't know this yet, positive should be letting go up negative pulling down
        mLeftClimberMotor.setIdleMode(IdleMode.kBrake);
        mRightClimberMotor.setIdleMode(IdleMode.kBrake);
        mTrapHookMotor.setIdleMode(IdleMode.kBrake);
        mLeftClimberMotor.setSmartCurrentLimit(40);
        mRightClimberMotor.setSmartCurrentLimit(40);
        mTrapHookMotor.setSmartCurrentLimit(40);
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

        NetworkLogger.initLog("Climber Matches State", true);
    }

    public enum ClimberState implements Subsystem.SubsystemState {
        IDLE, EXTEND, RETRACT
    }

    @Override
    public void genericPeriodic() {
        NetworkLogger.logData("Climber Matches State", matchesDesiredState());

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
            mRightClimberPID.reset();
        }

        double leftDutyCycle = 0; 
        double rightDutyCycle = 0;
        double trapHookDutyCycle = 0;

        if (dutyCycle < 0) {
            leftDutyCycle = dutyCycle  * 0.9;
            rightDutyCycle = dutyCycle + mRightClimberPID.calculate(
                mRightClimberMotor.getEncoder().getPosition(),
                mLeftClimberMotor.getEncoder().getPosition()
                );
            // trapHookDutyCycle = dutyCycle * 0.1; //temporary for testing negative would be for pulling down.
            // if (mLeftReverseLimitSwitch.isPressed() && mRightReverseLimitSwitch.isPressed()) { 
            //     mEngageTrapHook = true;

            // } 
            // if (mEngageTrapHook) {
            //     trapHookDutyCycle = -0.1 * dutyCycle; // temporay
            //    // trapHookDutyCycle = mTrapHookPID.calculate(
            //    //     mTrapHookMotor.getEncoder().getPosition(),
            //    //     ConfigMap.TRAP_HOOK_DOWN_POSITION
            //    // );
            // }
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
        // mTrapHookMotor.set(trapHookDutyCycle);
        // System.out.printf("trap dc: %.2f\n", trapHookDutyCycle);
    }

    public void manualTrapHooks(double dc) {
        final double TOLERANCE  = 0.1;
        if(Math.abs(dc) < TOLERANCE) {
            mTrapHookMotor.set(0);
            return;
        }

        mTrapHookMotor.set(dc);

        System.out.println("trap dc: " + dc + " trap encoder: " + mTrapHookMotor.getEncoder().getPosition());
    }

    public void pullTrapHooks() {
         //Trap hook motor set on reverse power to get the trap hooks to the encoder position that is all the way down
         //and keep it there with a pid, happens when the reverse limit switch is hit and the user is trying to pull down.
         //so probably not called here at all and done in ...
         
    }
    // public void twoThumbClimb(double dutyCycleLeft, double dutyCycleRight) {
    //     if (Math.abs(dutyCycleLeft) <= 0.1) {
    //         dutyCycleLeft = 0;
    //     } 
    //     if(Math.abs(dutyCycleRight) <= 0.1) {
    //         dutyCycleRight = 0;
    //     }

    //     dutyCycleLeft = dutyCycleLeft < 0 ? dutyCycleLeft: dutyCycleLeft * 0.7; 
    //     dutyCycleRight = dutyCycleRight < 0 ? dutyCycleRight: dutyCycleRight * 0.7; 

    //     mLeftClimberMotor.set(dutyCycleLeft);
    //     mRightClimberMotor.set(dutyCycleRight);
    // }
}
