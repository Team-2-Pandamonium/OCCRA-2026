package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.constants.RobotConstants;

public class Elevator {
  
  /**
   * This calculates how much the elevator has to move to get to the desired level
   * 
   * @apiNote 0 = ground
   * @apiNote 1 = 1st shelf
   * @apiNote 2 = 2nd shelf
   * @apiNote 3 = 3rd shelf
   * @param level
   * @param currHeight (in rotations)
   * @return <b>movDist<b> (in rotations)
   */
  public static double CalcDist(int level, double currHeight) {
    double desLevel = 0;
    switch (level) {
      case 0:
        desLevel = 0;
        break;
      case 1: // 1st shelf
        desLevel = RobotConstants.Level1;
        break;
      case 2: // 2nd shelf
        desLevel = RobotConstants.Level2;
        break;
      case 3: // 3rd shelf
        desLevel = RobotConstants.Level3;
        break;
      default: // else: return 0
        System.err.println("Error, invalid level number");
        break;
    }
    double movDist = InToRot(desLevel) - currHeight;
    return movDist;
  }

  /**
   * @param inches
   * @return <b>rotations<b>
   */
  public static double InToRot(double inches) {
    double cir = Math.PI * 2.256; // in inches
    double rot = inches / cir;
    rot *= 9; // gear ratio
    return rot;
  }

  /**
   * @param rot
   * @return <b>In<b>
   */
  public static double RotToIn(double rot) {
    rot /= 9; // gear ratio
    double cir = Math.PI * 2.256; // in inches
    double In = rot * cir;
    return In;
  }

  /**
  *
  * @param powered (use the motors to go to the 0 point or not)
  */
  public static void reset0(boolean powered) {
  if (powered) {
  while (Robot.CarrigeBottom.get()) {
  Robot.elevatorR.set(-0.3);
  }
  Robot.elevatorR.set(0);
  }
  if (!RobotConstants.bottEndstop) {
    Robot.elevatorEnc.setPosition(0);
  }
  
  }

}
