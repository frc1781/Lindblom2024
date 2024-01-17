package tech.team1781.autonomous;

public class RoutineOverException extends Exception {
   private String mName;

   public RoutineOverException(String routineName) {
    mName = routineName;
   } 

   @Override 
   public void printStackTrace() {
    System.out.printf("Routine: %s is over", mName);
   }
}
