package tech.team1781.subsystems;


import tech.team1781.utils.NetworkLogger;

public abstract class Subsystem {
   protected final String name;
   protected double currentTime;
   protected OperatingMode currentMode; 
   private final SubsystemState defaultState;
   private SubsystemState currentState;
   protected final NetworkLogger mNetworkLogger = new NetworkLogger();
   
   protected Subsystem(String _name, SubsystemState _defaultState) {
    name = _name;
    defaultState = _defaultState;
    currentState = defaultState;
    mNetworkLogger.log(getName(), currentTime);
   }

   public void setOperatingMode(OperatingMode mode) {
      currentMode = mode;
      init();
   }

   public abstract void genericPeriodic();

   public final String getName() {
      return name;
   }

   public final void feedStateTime(double sampledTime) {
    currentTime = sampledTime;
   }

   public final void setDesiredState(SubsystemState desiredState) {
      currentState = desiredState;
      mNetworkLogger.log(getName(), getState().toString());
   }
   
   public final SubsystemState getState() {
    return currentState;
   }

   public final void restoreDefault() {
      currentState = defaultState;
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
