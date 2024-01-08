package tech.team1781.subsystems;


public abstract class EESubsystem {
   protected final String name;
   protected double currentTime;
   private SubsystemState mCurrentState;
   
   protected EESubsystem(String _name) {
    name = _name;
   }

   public void genericInit() {
   }

   public void genericPeriodic() {
   }

   public final void feedStateTime(double sampledTime) {
    currentTime = sampledTime;
   }

   public final void setDesiredState(SubsystemState desiredState) {
    mCurrentState = desiredState;
   }

   public final SubsystemState getState() {
    return mCurrentState;
   }

   public abstract void getToState();

   public abstract boolean matchesDesiredState();

   public abstract void autonomousInit();

   public abstract void autonomousPeriodic();

   public abstract void teleopInit();

   public abstract void teleopPeriodic();

   public interface SubsystemState {

   }

}
