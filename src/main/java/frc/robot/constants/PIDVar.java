package frc.robot.constants;

public class PIDVar {
  // kP
  public static final double elevatorP = 0.01;
  public static final double manLeftP = 0.01;
  public static final double manRightP = 0.01;
  public static final double autonLeftP = 0.01;
  public static final double drvRightP = 0.01;
  // kI
  public static final double elevatorI = 0; // keep at 0 unless we REALLY NEED it becasue this can cause more problems
  public static final double manLeftI = 0; // keep at 0 unless we REALLY NEED it becasue this can cause more problems
  public static final double manRightI = 0; // keep at 0 unless we REALLY NEED it becasue this can cause more problems
  public static final double autonLeftI = 0;
  public static final double drvRightI = 0;
  // kD
  public static final double elevatorD = 0.01;
  public static final double manLeftD = 0.01;
  public static final double manRightD = 0.01;
  public static final double autonLeftD = 0.01;
  public static final double drvRightD = 0;
}
