package tech.team1781.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj.XboxController;

public class Climber extends Subsystem {

    private CANSparkMax motorLeft = new CANSparkMax(0, MotorType.kBrushless);
    private CANSparkMax motorRight = new CANSparkMax(1, MotorType.kBrushless);
    // Idea to index motors via for loop to prevent redundancy
    private CANSparkMax [] motors = {motorLeft, motorRight};
    private RelativeEncoder encoderLeft = motorLeft.getEncoder();
    private RelativeEncoder encoderRight = motorRight.getEncoder();
    private ClimberState mClimberState = ClimberState.IDLE;
    private ClimberState mDesiredClimberState = ClimberState.IDLE;
    private SparkLimitSwitch mDownLimitSwitchLeft = motorLeft.getReverseLimitSwitch(Type.kNormallyOpen);
    private SparkLimitSwitch mUpLimitSwitchLeft = motorLeft.getForwardLimitSwitch(Type.kNormallyOpen);
    private SparkLimitSwitch mDownLimitSwitchRight = motorRight.getReverseLimitSwitch(Type.kNormallyOpen);
    private SparkLimitSwitch mUpLimitSwitchRight = motorRight.getForwardLimitSwitch(Type.kNormallyOpen);

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
        if (encoderLeft.getPosition() > 0 && encoderRight.getPosition() > 0) {
            mDesiredClimberState = ClimberState.RETRACT;
        } else {
        mDesiredClimberState = ClimberState.IDLE;
        }

        getToState();

    }

    @Override
    public void getToState() {
        switch (mDesiredClimberState) {
            case IDLE:
                    motorLeft.set(0);
                    motorRight.set(0);
                break;

            case EXTEND:
                if (encoderLeft.getPosition() < 100) {
                    motorLeft.set(0.5);
                }
                 if (encoderRight.getPosition() < 100) {
                    motorRight.set(0.5);
                } else {
                    mDesiredClimberState = ClimberState.IDLE;
                }
                break;

            case RETRACT:
                if (encoderLeft.getPosition() > 0) {
                    motorLeft.set(-0.5);
                }
                if (encoderRight.getPosition() > 0) {
                    motorRight.set(-0.5);
                }
                else {
                    mDesiredClimberState = ClimberState.IDLE;
                }
                break;
        }   
    }

    @Override
    public boolean matchesDesiredState() {
    //     switch((ClimberState) getState()) {
    //         case IDLE:
    //             return motor.get() == 0; 
    //         case EXTEND:
    //             return motor.get() == .5; 
    //         case RETRACT:
    //             return motor.get() == -.5;

    // }
    return false;
}

public boolean extendedToMax() {
    // Will replace 100 with whatever our max ends up being
    // return encoder.getPosition() >= 100 || mUpLimitSwitch.isPressed();
    return false;
}





    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {

    }

}
