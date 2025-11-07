package frc.robot.constants;

public class PIDVar {
  // kP
  public static final double elevatorRP = 0.01;
  public static final double elevatorLP = 0.01;
  public static final double manLeftP = 0.01;
  public static final double manRightP = 0.01;
  // kI
  public static final double elevatorRI = 0; // keep at 0 unless we REALLY NEED it becasue this can cause more problems
  public static final double elevatorLI = 0; // keep at 0 unless we REALLY NEED it becasue this can cause more problems
  public static final double manLeftI = 0; // keep at 0 unless we REALLY NEED it becasue this can cause more problems
  public static final double manRightI = 0; // keep at 0 unless we REALLY NEED it becasue this can cause more problems
  // kD
  public static final double elevatorRD = 0.01;
  public static final double elevatorLD = 0.01;
  public static final double manLeftD = 0.01;
  public static final double manRightD = 0.01;
}
