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
   * @return <b>movDist<b>
   */
  public static double calcDist(int level,double currHeight) {
    double desLevel = 0;
    switch (level) {
      case 0:
        desLevel = 0;
        break;
      case 1: // 1st shelf
        desLevel = RobotConstants.Level1GrdOff;
        break;
      case 2: // 2nd shelf
        desLevel = RobotConstants.Level2GrdOff;
        break;
      case 3: // 3rd shelf
        desLevel = RobotConstants.Level3GrdOff;
        break;
      case 4: // 1st shelf but rotate
        desLevel = RobotConstants.Level1GrdOff - 2;
        break;
      case 5: // 2nd shelf but rotate
        desLevel = RobotConstants.Level2GrdOff - 2;
        break;
      case 6: // 3rd shelf but rotate+
        desLevel = RobotConstants.Level3GrdOff - 2;
        break;
      case 7: // human player station
        desLevel = RobotConstants.humanPlayerGrdOff;
        break;
      default: // else: return 0
        System.err.println("Error, invalid level number");
        break;
    }
    double movDist = desLevel - currHeight;
    return movDist;
  }

  /**
   * 
   * @param inches
   * @return <b>rotations<b>
   */
  public static double inchesToRotations(double inches) {
    return inches * 0.761475409836066; // determined experimentally
  }

  /**
   * 
   * @param rot
   * @return <b>In<b>
   */
  public static double RottoIn(double rot) {
    return rot * 1.31324004305705; // determined experimentally
  }

  /**
   * 
   * @param desLevel
   * @param currHeight
   * @return <b>AmtRot<b>
   */
  public static double CalcRot(int desLevel,double currHeight){
    double deltaHeight=calcDist(desLevel, currHeight);
    double rot=inchesToRotations(deltaHeight);
    return rot;
  }

  // /**
  // *
  // * @param powered (use the motors to go to the 0 point or not)
  // */
  // public static void reset0(boolean powered) {
  // if (powered) {
  // while (Robot.CarrigeBottom.get()) {
  // Robot.elevatorR.set(-0.5);
  // }
  // Robot.elevatorR.set(0);
  // }
  // Robot.elevatorEnc.setPosition(0);
  // }

}
