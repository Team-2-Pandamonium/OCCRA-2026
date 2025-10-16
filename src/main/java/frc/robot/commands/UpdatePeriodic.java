package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class UpdatePeriodic {
    /**
     * Gets the value from the controllers and updates it in the code
     */
    public static void updateControllerInputs() {

        // if any of these are below the deadzone, it equals zero. abs so the controller
        // can go negative
        if (Math.abs(frc.robot.Robot.controller_1.getLeftY()) >= RobotConstants.deadzone) {
            RobotConstants.leftStick = frc.robot.Robot.controller_1.getLeftY();
        } else {
            RobotConstants.leftStick = 0;
        }

        if (Math.abs(frc.robot.Robot.controller_1.getRightY()) >= RobotConstants.deadzone) {
            RobotConstants.rightStick = frc.robot.Robot.controller_1.getRightY();
        } else {
            RobotConstants.rightStick = 0;
        }

        if (frc.robot.Robot.controller_1.getLeftTriggerAxis() >= RobotConstants.deadzone) {
            RobotConstants.leftTrigger = frc.robot.Robot.controller_1.getLeftTriggerAxis();
        } else {
            RobotConstants.leftTrigger = 0;
        }

        if (frc.robot.Robot.controller_1.getRightTriggerAxis() >= RobotConstants.deadzone) {
            RobotConstants.rightTrigger = frc.robot.Robot.controller_1.getRightTriggerAxis();
        } else {
            RobotConstants.rightTrigger = 0;
        }

        // booleans, so no deadzone needed
        RobotConstants.leftBumper = frc.robot.Robot.controller_1.getLeftBumper();
        RobotConstants.rightBumper = frc.robot.Robot.controller_1.getRightBumper();
        RobotConstants.bButton = frc.robot.Robot.controller_1.getBButton();
        RobotConstants.xButton = frc.robot.Robot.controller_1.getXButton();
        RobotConstants.aButton = frc.robot.Robot.controller_1.getAButton();
        RobotConstants.beamSensor = frc.robot.Robot.beamBreak.get();

    }

}