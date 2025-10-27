package frc.robot.constants;

public class PIDVar {
  // kP
  public static double elevatorRP = 0.01;
  public static double elevatorLP = 0.01;
  // kI
  public static double elevatorRI = 0; // keep at 0 unless we REALLY NEED it becasue this can cause more problems
  public static double elevatorLI = 0; // keep at 0 unless we REALLY NEED it becasue this can cause more problems
  // kD
  public static double elevatorRD = 0.01;
  public static double elevatorLD = 0.01;
}
