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
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getToState'");
    }

    @Override
    public boolean matchesDesiredState() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'matchesDesiredState'");
    }


    @Override
    public void autonomousPeriodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'autonomousPeriodic'");
    }

    @Override
    public void teleopPeriodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'teleopPeriodic'");
    }

    @Override
    public void genericPeriodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'genericPeriodic'");
    }

    @Override
    public void init() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'init'");
    }
    
}
