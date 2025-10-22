package frc.robot.constants;

import com.revrobotics.RelativeEncoder;

public class PIDVar {
  // kP
  public static double manShortP = 0.01;
  public static double manLongP = 0.01;
  public static double elevatorRP = 0.01;
  public static double elevatorLP = 0.01;
  public static double right1P = 0.01;
  public static double right2P = 0.01;
  public static double left1P = 0.01;
  public static double left2P = 0.01;
  // kI
  public static double manShortI = 0; // keep at 0 unless we REALLY NEED it becasue this can cause more problems
  public static double manLongI = 0; // keep at 0 unless we REALLY NEED it becasue this can cause more problems
  public static double elevatorRI = 0; // keep at 0 unless we REALLY NEED it becasue this can cause more problems
  public static double elevatorLI = 0; // keep at 0 unless we REALLY NEED it becasue this can cause more problems
  public static double right1I = 0; // keep at 0 unless we REALLY NEED it becasue this can cause more problems
  public static double right2I = 0; // keep at 0 unless we REALLY NEED it becasue this can cause more problems
  public static double left1I = 0; // keep at 0 unless we REALLY NEED it becasue this can cause more problems
  public static double left2I = 0; // keep at 0 unless we REALLY NEED it becasue this can cause more problems
  // kD
  public static double manShortD = 0.01;
  public static double manLongD = 0.01;
  public static double elevatorRD = 0.01;
  public static double elevatorLD = 0.01;
  public static double right1D = 0.01;
  public static double right2D = 0.01;
  public static double left1D = 0.01;
  public static double left2D = 0.01;
}
