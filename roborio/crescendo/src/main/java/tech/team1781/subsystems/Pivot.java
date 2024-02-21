package tech.team1781.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Counter;

// import tech.team1781.subsystems.Pivot.PivotState;

public class Pivot extends Subsystem{
    
    private CANSparkMax mPivotMotor;
    private Counter motorCounter;
    private int mPosition;
    private boolean movingUp;

    public Pivot() {
        super("Pivot", PivotState.IDLE);
        mPivotMotor = new CANSparkMax(0, CANSparkMax.MotorType.kBrushless);
        motorCounter = new Counter(new DigitalInput(1));
        movingUp = false;
    }

    public enum PivotState implements Subsystem.SubsystemState {
        IDLE,
        PIVOT_UP,
        PIVOT_DOWN
    }

    public void init() {

    }

    public void genericPeriodic() {

    }

    public void teleopPeriodic() {

    }

    public void autonomousPeriodic() {

    }

    public void getToState() {
        updatePosition();
    }

    public boolean matchesDesiredState() {
        return false;
    } 

    public void updatePosition() {
        if (movingUp) {
            mPosition += 1;
        } else {
            mPosition -= 1;
        }
    }

}
