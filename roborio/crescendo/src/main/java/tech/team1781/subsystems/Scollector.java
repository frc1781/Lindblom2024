package tech.team1781.subsystems;

public class Scollector extends Subsystem {

    public Scollector() {
        super("Scollector", ScollectorState.IDLE);
    }

    public enum ScollectorState implements Subsystem.SubsystemState {
        COLLECT, SHOOT, AMP, AUTO_AIM_SHOOT, IDLE
    }

    @Override
    public void genericPeriodic() {
    }

    @Override
    public void init() {
    }

    @Override
    public void getToState() {
        System.out.println(getState().toString());
    }

    @Override
    public boolean matchesDesiredState() {
        switch ((ScollectorState) getState()) {
            case COLLECT:
            return false;
            case SHOOT: 
            return false; 
            default: 
            return false;
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }

}
