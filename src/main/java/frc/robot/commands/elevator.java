package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.constants.RobotConstants;

/*
 * TODO:
 * - pid loop for elevator bc we actaully have ecoders :)
 */
public class Elevator {
  
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
  public static double calcDist(int level,double currHeight) {
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
    double movDist = desLevel - currHeight;
    return movDist;
  }

  public static double inchesToRotations(double inches) {
    double rotations;
    rotations = (inches * 25.4) / 20;
    return rotations;
    // need rps (rotation per second) at max cause number of rotations needed
    // converted to seconds needed, then jst a wait statement
    // btw 1 rotation is 20 mm but maybe innacurate due to vectors with belts (jst
    // multiplied the number of teeth by the dist between them) ;-;
  }
  public static double RottoIn(double rot) {
    double in;
    in=(rot*20)/25.4;
    return in;
    // need rps (rotation per second) at max cause number of rotations needed
    // converted to seconds needed, then jst a wait statement
    // btw 1 rotation is 20 mm but maybe innacurate due to vectors with belts (jst
    // multiplied the number of teeth by the dist between them) ;-;
  }

  /**
   * Acts like the P part of PID control loop
   * @param desLevel
   * @param currHeight
   * @return <b>MotorSpeed<b>
   */
  public static double dumbCalcMotSpd(int desLevel,double currHeight){
    double deltaHeight=calcDist(desLevel, currHeight);
    double rot=inchesToRotations(deltaHeight);
    return rot/76.2;
  }

  public static void reset0(){
    //go down until we see the voltage of motors spike and then we know its at the bottom
    double volt=0;
    double prevVolt=0;
    final double voltThreshold=3;
    while (!(Math.abs(volt-prevVolt)>voltThreshold)) {
      Robot.elevatorL.set(-0.75);
      Robot.elevatorR.set(-0.75);
    }
    Robot.elevatorL.set(0);
    Robot.elevatorR.set(0);
    Robot.encoder.reset();
    
  }
}
