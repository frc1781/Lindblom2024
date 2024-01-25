package tech.team1781.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import tech.team1781.ConfigMap;

//EXAMPLE SUBSYSTEM, NOT FOR ACTUAL BOT
public class Scollector extends Subsystem {
    private CANSparkMax mCollectorMotor = new CANSparkMax(ConfigMap.COLLECTOR_MOTOR, CANSparkMax.MotorType.kBrushless);
    private TimeOfFlight mTimeOfFlight = new TimeOfFlight(ConfigMap.COLLECTOR_TOF);
    private CANSparkMax mTopShooterMotor = new CANSparkMax(ConfigMap.TOP_SHOOTER_MOTOR,
            CANSparkMax.MotorType.kBrushless);
    private CANSparkMax mShooterMotor = new CANSparkMax(ConfigMap.SHOOTER_MOTOR, CANSparkMax.MotorType.kBrushless);

    public Scollector() {
        super("Scollector", ScollectorState.IDLE);
    }

    public enum ScollectorState implements SubsystemState {
        IDLE, COLLECT, SPIT
    }

    @Override
    public void genericPeriodic() {
    }

    @Override
    public void init() {
    }

    public boolean hasNote() {
        if (mTimeOfFlight.getRange() < 10) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void getToState() {
        switch ((ScollectorState) getState()) {
            case IDLE:
                closeCollector();
                mCollectorMotor.set(0);
                break;
            case COLLECT:
                openCollector();
                mCollectorMotor.set(1);
                if (hasNote() == true) {
                    mCollectorMotor.set(0);
                }
                break;
            case SPIT:
                openCollector();
                mCollectorMotor.set(-1);
                break;
        }
    }

    @Override
    public boolean matchesDesiredState() {
        switch ((ScollectorState) getState()) {
            case IDLE:
                return mCollectorMotor.get() == 0;
            case COLLECT:
                return hasNote();
            case SPIT:
                return mCollectorMotor.get() == -1;
            default:
                return true;
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }

    private void openCollector() {
    }

    private void closeCollector() {
    }

}
