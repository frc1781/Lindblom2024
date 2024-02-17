package tech.team1781.subsystems;
import tech.team1781.utils.NetworkLogger;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import tech.team1781.ConfigMap;
import tech.team1781.subsystems.Scollector.ScollectorState;

public class Climber extends Subsystem {
    private CANSparkMax mLeftClimberMotor = new CANSparkMax(ConfigMap.LEFT_CLIMBER_MOTOR,
            CANSparkMax.MotorType.kBrushless);
    private CANSparkMax mRightClimberMotor = new CANSparkMax(ConfigMap.RIGHT_CLIMBER_MOTOR,
            CANSparkMax.MotorType.kBrushless);
    private NetworkLogger mNetworkLogger = new NetworkLogger();

    public Climber() {
        super("Climber", ClimberState.IDLE);
        mLeftClimberMotor.setInverted(true);
        mRightClimberMotor.setInverted(false);
        mLeftClimberMotor.setIdleMode(IdleMode.kBrake);
        mRightClimberMotor.setIdleMode(IdleMode.kBrake);
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

        switch ((ClimberState) getState()) {
            case IDLE:
                leftDC = 0;
                rightDC = 0;
                break;
            case EXTEND:
                leftDC = 0.5;
                rightDC = 0.5;
                break;
            case RETRACT:
                leftDC = -0.5;
                rightDC = -0.5;
                break;
        }

        // System.out.printf("left dc: %.2f right dc: %.2f\n",
        //     leftDC,
        //     rightDC
        // );       
        mLeftClimberMotor.set(leftDC);
        mRightClimberMotor.set(rightDC);     
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

}

