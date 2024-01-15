package tech.team1781.subsystems;

public class Climber extends Subsystem{

    public Climber() {
        super("Climber", ClimberState.IDLE);
    }

    public enum ClimberState implements Subsystem.SubsystemState{
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
        System.out.println("i'm " + getState().toString());
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
