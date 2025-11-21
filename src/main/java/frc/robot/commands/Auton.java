package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;

public class Auton extends SubsystemBase {

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
    public void goFwd(double rot) {
        double encLOffset = Robot.drvLEnc.getPosition();
        double encROffset = Robot.drvREnc.getPosition();

        Robot.left1.set(0);
        Robot.right1.set(0);

        while (Math.abs(Robot.drvLEnc.getPosition()-encLOffset) < Math.abs(rot) && Math.abs(Robot.drvREnc.getPosition()-encROffset) < Math.abs(rot)) {
            if (rot > 0) {
                    Robot.left1.set(-0.25 * ((Math.abs(Robot.drvLEnc.getPosition()-encLOffset) - Math.abs(rot))/((Math.abs(Robot.drvLEnc.getPosition()-encLOffset) - Math.abs(rot))+1)));
                    Robot.right1.set(-0.25 * ((Math.abs(Robot.drvREnc.getPosition()-encROffset) - Math.abs(rot))/((Math.abs(Robot.drvREnc.getPosition()-encROffset) - Math.abs(rot))+1)));
            } else {
                    Robot.left1.set(0.25 * ((Math.abs(Robot.drvLEnc.getPosition()-encLOffset) - Math.abs(rot))/((Math.abs(Robot.drvLEnc.getPosition()-encLOffset) - Math.abs(rot))+1)));
                    Robot.right1.set(0.25 * ((Math.abs(Robot.drvREnc.getPosition()-encROffset) - Math.abs(rot))/((Math.abs(Robot.drvREnc.getPosition()-encROffset) - Math.abs(rot))+1)));}
            }
    }


    /**
     * IMPORTANT: input "L" or "R" for left and right respectively
     * @param LR
     * @return makes the robot turn left or right
     */
    public void turnLR(String LR) {
        BooleanSupplier leftReq = () -> Robot.gyro.getAngle() < 90;
        BooleanSupplier rightReq = () -> Robot.gyro.getAngle() > -90;

        if (LR == "L") {
            while (Robot.gyro.getAngle() < 90) {
                Robot.right1.set(-0.25);
                Robot.left1.set(0.25);
            }
        } else if (LR == "R") {
            while (Robot.gyro.getAngle() > -90) {
                Robot.right1.set(0.25);
                Robot.left1.set(-0.25);
            }
            } else {
                System.out.print("U DIDN'T INPUT L OR R (uppercase)");
            }
        }

    public void goToSpecificElevatorLevel(int level) {
        while (Math.abs(Elevator.CalcDist(level, Robot.elevatorEnc.getPosition()) - Robot.elevatorEnc.getPosition()) >= 0.5) {
            Robot.elevatorR.set(-(Elevator.CalcDist(level, Robot.elevatorEnc.getPosition())/(76.25)));
        }
    }

    public void intake() {
        Robot.manLeft.set(1);
        Robot.manRight.set(1);
    }

    public void outake() {
        Robot.manLeft.set(-1);
        Robot.manRight.set(-1);
    }

    }


