package frc.robot.commands;

import frc.robot.constants.RobotConstants;

public class elevator {

  /**
   * This calculates how much the elevator has to move to get to the desired level
   * 
   * @apiNote 1=1st shelf
   * @apiNote 2=2nd shelf
   * @apiNote 3=3rd shelf
   * @apiNote 4=1st shelf indexing
   * @apiNote 5=2nd shelf indexing
   * @apiNote 6=3rd shelf indexing
   * @apiNote 7=HP station
   * @param level
   * @return <b>movDist<b>
   */
  public static double calcDist(int level) {
    double desLevel = 0;
    switch (level) {
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
        desLevel = RobotConstants.Level1 - 2;
        break;
      case 5: // 2nd shelf but rotate
        desLevel = RobotConstants.Level2 - 2;
        break;
      case 6: // 3rd shelf but rotate+
        desLevel = RobotConstants.Level3 - 2;
        break;
      case 7: // human player station
        desLevel = RobotConstants.humanPlayer;
        break;
      default: // else: return 0
        break;
    }
    double movDist = desLevel - RobotConstants.elevatorHeight;
    return movDist;
  }

  public static double inchesToRotations(int inches) {
    double rotations = 0;
    rotations = (inches * 25.4) / 20;
    return rotations;
    // need rps (rotation per second) at max cause number of rotations needed
    // converted to seconds needed, then jst a wait statement
  }

}
