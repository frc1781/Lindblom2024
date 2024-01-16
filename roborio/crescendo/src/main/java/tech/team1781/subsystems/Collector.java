package tech.team1781.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import tech.team1781.ConfigMap;

//EXAMPLE SUBSYSTEM, NOT FOR ACTUAL BOT
public class Collector extends Subsystem{
    
    private DoubleSolenoid mLeftSolenoid = new DoubleSolenoid(ConfigMap.FIRST_PCM_ID,
        PneumaticsModuleType.CTREPCM,
        ConfigMap.COLLECTOR_LEFT_OPEN,
        ConfigMap.COLLECTOR_LEFT_CLOSE
    );

    private DoubleSolenoid mRightSolenoid = new DoubleSolenoid(ConfigMap.FIRST_PCM_ID,
        PneumaticsModuleType.CTREPCM,
        ConfigMap.COLLECTOR_RIGHT_OPEN,
        ConfigMap.COLLECTOR_RIGHT_CLOSE
    );

    private PWMTalonSRX mCollectorMotor = new PWMTalonSRX(ConfigMap.COLLECTOR_MOTOR);


    public Collector() {
        super("Collector", CollectorState.IDLE);
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
                return mLeftSolenoid.get() == DoubleSolenoid.Value.kReverse &&
                    mRightSolenoid.get() == DoubleSolenoid.Value.kReverse &&
                    mCollectorMotor.get() == 0;
            case COLLECT:
                return mLeftSolenoid.get() == DoubleSolenoid.Value.kForward &&
                    mRightSolenoid.get() == DoubleSolenoid.Value.kForward &&
                    mCollectorMotor.get() == 1;
            case SPIT:
                return mLeftSolenoid.get() == DoubleSolenoid.Value.kForward &&
                    mRightSolenoid.get() == DoubleSolenoid.Value.kForward &&
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
        mLeftSolenoid.set(DoubleSolenoid.Value.kForward);
        mRightSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    private void closeCollector() {
        mLeftSolenoid.set(DoubleSolenoid.Value.kReverse);
        mRightSolenoid.set(DoubleSolenoid.Value.kReverse);
    }
    
}
