package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.constants.RobotConstants;

public class Auton {

    /**
     * 
     * @param linVel
     * @return <b>motRot<b>
     */
    public static double distToRot(double linDist) {
        double wheelRot = linDist / (Math.PI * 6);
        return wheelRot * 8.46;
    }

    /**
     * 
     * @param rot
     * @throws myCat
     * @throws urMom
     * @return true, also makes the drivetrain move that many rotations on both motors, going forward/backward
     */
    public static boolean goFwd(double rot) {
        double encLOffset = Robot.drvLEnc.getPosition();
        double encROffset = Robot.drvREnc.getPosition();
        while (Math.abs(Robot.drvLEnc.getPosition()-encLOffset) < Math.abs(rot)
                && Math.abs(Robot.drvREnc.getPosition()-encROffset) < Math.abs(rot)) {
            if (rot > 0) {
                Robot.left1.set(-RobotConstants.autonSpeed);
                Robot.right1.set(-RobotConstants.autonSpeed);
            } else {
                Robot.left1.set(RobotConstants.autonSpeed);
                Robot.right1.set(RobotConstants.autonSpeed);
            }
        }
        Robot.left1.set(0);
        Robot.right1.set(0);
        return true;
    }

}
