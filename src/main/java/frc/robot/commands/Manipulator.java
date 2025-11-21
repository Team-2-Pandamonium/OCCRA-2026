package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;

public class Manipulator extends SubsystemBase {

    /**
     * 
     * @param drvRot
     * @return <b>linVel<b> (inches)
     */
    public static double drvRotLinVel(double drvMotRot){
        double wheelRot=drvMotRot/8.46;
        return wheelRot*(Math.PI*6);
    }

    /**
     * 
     * @param linVel
     * @return <b>drvRot<b>
     */

    /**
     * 
     * @param degrees to turn
     * @param seconds for degrees to turn
     * @return turning speed of both motors, one negative and one positive, for drivetrain
     */
    public static double degAndSecsToDrvSpeed(double degrees, double seconds) {
        return 13.5*(degrees/seconds)*(Math.PI/180);
        // (Width of the robot: 27, divided by 2, since 2 seperate sides) times (degrees/sec)*(pi/180), which is the equation for radians per sec
    }


    /**
     * @apiNote divide by circumference (diameter=4), and multiply by gear ratios (4)
     * @param LinVel
     * @return <b>manRot<b> (rot/sec)
     */
    public static double LinVeltoManRot(double LinVel){
        double wheelRot=LinVel/(Math.PI*4);
        return wheelRot*4;
    }

    public void SetManipulators(double Speed) {
            Robot.manLeft.set(Speed);
            Robot.manRight.set(Speed);
    }
    
    public void RandomWeirdThingThatOperatorBButtonDoes() {
            Robot.manLeft.set(1);
            Robot.manRight.set(1);
    }
}