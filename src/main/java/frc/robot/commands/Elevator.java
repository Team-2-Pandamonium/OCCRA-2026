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
   * @apiNote 4 = 1st shelf indexing
   * @apiNote 5 = 2nd shelf indexing
   * @apiNote 6 = 3rd shelf indexing
   * @apiNote 7 = HP station
   * @param level
   * @param currHeight (in rotations)
   * @return <b>movDist<b>
   */
  public static double CalcRot(int level,double currHeight) {
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
      case 4: // 1st shelf but rotate
        desLevel = RobotConstants.Level1 - RobotConstants.vertIndex;
        break;
      case 5: // 2nd shelf but rotate
        desLevel = RobotConstants.Level2 - RobotConstants.vertIndex;
        break;
      case 6: // 3rd shelf but rotate+
        desLevel = RobotConstants.Level3 - RobotConstants.vertIndex;
        break;
      case 7: // human player station
        desLevel = RobotConstants.humanPlayer;
        break;
      default: // else: return 0
        System.err.println("Error, invalid level number");
        break;
    }
    double movDist = desLevel - currHeight;
    return movDist;
  }

  // /**
  // * @deprecated
  // * @param inches
  // * @return <b>rotations<b>
  // */
  // public static double inchesToRotations(double inches) {
  // return inches * 0.761475409836066; // determined experimentally
  // }

  // /**
  // * @deprecated
  // * @param rot
  // * @return <b>In<b>
  // */
  // public static double RottoIn(double rot) {
  // return rot * 1.31324004305705; // determined experimentally
  // }

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
