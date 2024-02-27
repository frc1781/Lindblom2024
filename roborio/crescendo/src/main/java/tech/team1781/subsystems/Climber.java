package tech.team1781.subsystems;
import tech.team1781.utils.NetworkLogger;
import com.revrobotics.CANSparkMax;
import tech.team1781.ConfigMap;

public class Climber extends Subsystem {
    private CANSparkMax mLeftClimberMotor = new CANSparkMax(ConfigMap.LEFT_CLIMBER_MOTOR,
            CANSparkMax.MotorType.kBrushless);
    private CANSparkMax mRightClimberMotor = new CANSparkMax(ConfigMap.RIGHT_CLIMBER_MOTOR,
            CANSparkMax.MotorType.kBrushless);

    public Climber() {
        super("Climber", ClimberState.IDLE);
        mLeftClimberMotor.setInverted(true);
        mRightClimberMotor.setInverted(false);
    }

    public enum ClimberState implements Subsystem.SubsystemState {
        IDLE, EXTEND, RETRACT
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

    public void manualClimb(double dutyCycle) {
        if(Math.abs(dutyCycle) <= 0.1) {
            dutyCycle = 0;
        }

        mLeftClimberMotor.set(dutyCycle);
        mRightClimberMotor.set(dutyCycle);
    }

}

