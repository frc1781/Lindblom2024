package tech.team1781.subsystems;

import tech.team1781.utils.NetworkLogger;

public abstract class Subsystem {
   protected final String name;
   protected double currentTime;
   protected OperatingMode currentMode; 
   private final SubsystemState defaultState;
   protected SubsystemState currentState;
   protected final NetworkLogger mNetworkLogger = new NetworkLogger();

   protected Subsystem(String _name, SubsystemState _defaultState) {
    name = _name;
    defaultState = _defaultState;
    currentState = defaultState;
    mNetworkLogger.log(getName(), currentState.toString());
   }

   public void setOperatingMode(OperatingMode mode) {
      currentMode = mode;
      System.out.println(name + " initialized into operating mode " + mode.toString());
      init();
   }

   public abstract void genericPeriodic();

   public final String getName() {
      return name;
   }

   public final void feedStateTime(double sampledTime) {
      currentTime = sampledTime;
   }

   public void setDesiredState(SubsystemState desiredState) {
      if (desiredState == currentState) {
         return;
      }
      currentState = desiredState;
      mNetworkLogger.log(getName(), getState().toString());
      System.out.println("Changing " + name +  "'s state to " + desiredState);
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
