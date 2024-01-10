package tech.team1781.subsystems;


public abstract class SubsystemController {
   protected final String name;
   protected double currentTime;
   protected OperatingMode currentMode; 

   private SubsystemState mCurrentState;
   
   protected SubsystemController(String _name) {
    name = _name;
   }

   public void setOperatingMode(OperatingMode mode) {
      currentMode = mode;
      init();
   }

   public abstract void genericPeriodic();

   public final void feedStateTime(double sampledTime) {
    currentTime = sampledTime;
   }

   public final void setDesiredState(SubsystemState desiredState) {
    mCurrentState = desiredState;
   }

   public final SubsystemState getState() {
    return mCurrentState;
   }

   public abstract void init();

   public abstract void getToState();

   public abstract boolean matchesDesiredState();

   public abstract void autonomousPeriodic();

   public abstract void teleopPeriodic();

   public interface SubsystemState {

   }

   public enum OperatingMode {
      DISABLED, TELEOP, AUTONOMOUS, SIMULATION, TEST
   }

}
