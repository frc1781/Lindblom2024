package tech.team1781.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj.XboxController;

public class Climber extends Subsystem {

    private CANSparkMax motor = new CANSparkMax(0, MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();
    private ClimberState mClimberState = ClimberState.IDLE;
    private ClimberState mDesiredClimberState = ClimberState.IDLE;
    private SparkLimitSwitch mDownLimitSwitch = motor.getReverseLimitSwitch(Type.kNormallyOpen);
    private SparkLimitSwitch mUpLimitSwitch = motor.getForwardLimitSwitch(Type.kNormallyOpen);

    public Climber() {
        super("Climber", ClimberState.IDLE);
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
        switch (mDesiredClimberState) {
            case IDLE:
                    motor.set(0);
                break;

            case EXTEND:
                if (encoder.getPosition() < 100) {
                    motor.set(0.5);
                }
                break;

            case RETRACT:
                if (encoder.getPosition() > 0) {
                    motor.set(-0.5);
                }
                break;
        }
    }

    @Override
    public boolean matchesDesiredState() {
        switch((ClimberState) getState()) {
            case IDLE:
                return motor.get() == 0; 
            case EXTEND:
                return motor.get() == .5; 
            case RETRACT:
                return motor.get() == -.5;

    }return false;
}

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {

    }

}
