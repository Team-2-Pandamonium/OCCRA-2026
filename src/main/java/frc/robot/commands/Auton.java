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
    public Command goFwd(double rot) {
        double encLOffset = Robot.drvLEnc.getPosition();
        double encROffset = Robot.drvREnc.getPosition();

        Robot.left1.set(0);
        Robot.right1.set(0);

        BooleanSupplier whileCondition = () -> (Math.abs(Robot.drvLEnc.getPosition()-encLOffset) < Math.abs(rot)
        && Math.abs(Robot.drvREnc.getPosition()-encROffset) < Math.abs(rot));

        if (rot > 0) {
            return this.run(() -> 
                {Robot.left1.set(-RobotConstants.autonSpeed * ((Math.abs(Robot.drvLEnc.getPosition()-encLOffset) - Math.abs(rot))/((Math.abs(Robot.drvLEnc.getPosition()-encLOffset) - Math.abs(rot))+1)));
                Robot.right1.set(-RobotConstants.autonSpeed * ((Math.abs(Robot.drvREnc.getPosition()-encROffset) - Math.abs(rot))/((Math.abs(Robot.drvREnc.getPosition()-encROffset) - Math.abs(rot))+1)));})
                .onlyWhile(whileCondition);
        } else {
            return this.run(() -> 
                {Robot.left1.set(RobotConstants.autonSpeed * ((Math.abs(Robot.drvLEnc.getPosition()-encLOffset) - Math.abs(rot))/((Math.abs(Robot.drvLEnc.getPosition()-encLOffset) - Math.abs(rot))+1)));
                Robot.right1.set(RobotConstants.autonSpeed * ((Math.abs(Robot.drvREnc.getPosition()-encROffset) - Math.abs(rot))/((Math.abs(Robot.drvREnc.getPosition()-encROffset) - Math.abs(rot))+1)));})
                .onlyWhile(whileCondition);
        }

    }

    /**
     * IMPORTANT: input "L" or "R" for left and right respectively
     * @param LR
     * @return makes the robot turn left or right
     */
    public Command turnLR(String LR) {
        BooleanSupplier leftReq = () -> Robot.gyro.getAngle() < 90;
        BooleanSupplier rightReq = () -> Robot.gyro.getAngle() > -90;

        if (LR == "L") {
            return this.run(() -> {
            Robot.right1.set(-RobotConstants.autonSpeed);
            Robot.left1.set(RobotConstants.autonSpeed);})
            .onlyWhile(leftReq)
            .andThen(() -> {Robot.left1.set(0); Robot.right1.set(0);});
        } else if (LR == "R") {
            return this.run(() -> {
            Robot.right1.set(RobotConstants.autonSpeed);
            Robot.left1.set(-RobotConstants.autonSpeed);})
            .onlyWhile(rightReq)
            .andThen(() -> {Robot.left1.set(0); Robot.right1.set(0);});
            } else {
                return this.run(() -> {System.out.print("U DIDN'T INPUT L OR R (uppercase)");});
            }
        }

    public Command goToSpecificElevatorLevel(int level) {
        RobotConstants.elevatorOutput = (Elevator.CalcDist(level, RobotConstants.elevatorRotHeight)/(RobotConstants.elevatorMaxRot));
        BooleanSupplier Condition = () -> Math.abs(Elevator.CalcDist(level, RobotConstants.elevatorRotHeight) - Robot.elevatorEnc.getPosition()) >= 0.5;
        return this.run(() -> {Robot.elevatorR.set(-RobotConstants.elevatorOutput);})
        .onlyWhile(Condition)
        .andThen(() -> {Robot.elevatorR.set(0);});
    }

    public Command outake() {
        return this.run(() -> {Robot.manRight.set(-RobotConstants.manRightOutput); Robot.manLeft.set(-RobotConstants.manLeftOutput);});
    }

    public Command intake() {
        return this.run(() -> {Robot.manRight.set(RobotConstants.manRightOutput); Robot.manLeft.set(RobotConstants.manLeftOutput);});
    }
}

