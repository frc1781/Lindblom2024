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
        switch ((ClimberState) getState()) {
            case IDLE:
                mLeftClimberMotor.set(0);
                mRightClimberMotor.set(0);
                break;
            case EXTEND:
                mLeftClimberMotor.set(1);
                mRightClimberMotor.set(1);
                // mRightClimberMotor.follow(mRightClimberMotor);
                break;
            case RETRACT:
                mLeftClimberMotor.set(-1);
                mRightClimberMotor.set(-1);
                break;
        }
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
