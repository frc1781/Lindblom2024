package tech.team1781.subsystems;

public class DriveSystem extends Subsystem{

    public DriveSystem() {
        super("Drive System");
        //TODO Auto-generated constructor stub
    }

    public enum DriveSystemState implements Subsystem.SubsystemState {
        DRIVE_SETPOINT, 
        DRIVE_TRAJECTORY,
        DRIVE_MANUAL
    } 

    @Override
    public void getToState() {
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

    @Override
    public void genericPeriodic() {
    }

    @Override
    public void init() {
    }
    
}
