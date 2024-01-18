package tech.team1781.subsystems;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import tech.team1781.ConfigMap;

//EXAMPLE SUBSYSTEM, NOT FOR ACTUAL BOT
public class Scollector extends Subsystem{

    private PWMTalonSRX mCollectorMotor = new PWMTalonSRX(ConfigMap.COLLECTOR_MOTOR);


    public Scollector() {
        super("Scollector", CollectorState.IDLE);
    }

    public enum CollectorState implements SubsystemState {
        IDLE, COLLECT, SPIT
    }

    @Override
    public void genericPeriodic() {

    }

    @Override
    public void init() {
    }

    @Override
    public void getToState() {
        switch((CollectorState) getState()) {
            case IDLE:
                closeCollector();
                mCollectorMotor.set(0);
            break;
            case COLLECT:
                openCollector();
                mCollectorMotor.set(1);
            break;
            case SPIT:
                openCollector();
                mCollectorMotor.set(-1);
            break;
        }
    }

    @Override
    public boolean matchesDesiredState() {
        switch((CollectorState) getState()) {
            case IDLE:
                return 
                    mCollectorMotor.get() == 0;
            case COLLECT:
                return 
                    mCollectorMotor.get() == 1;
            case SPIT:
                return 
                    mCollectorMotor.get() == -1;
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

