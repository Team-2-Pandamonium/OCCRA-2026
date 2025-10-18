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
        if (Math.abs(frc.robot.Robot.DRIV_CONTROLLER.getLeftY()) >= RobotConstants.deadzone) {
            RobotConstants.leftStick = frc.robot.Robot.DRIV_CONTROLLER.getLeftY();
        } else {
            RobotConstants.leftStick = 0;
        }

        if (Math.abs(frc.robot.Robot.DRIV_CONTROLLER.getRightY()) >= RobotConstants.deadzone) {
            RobotConstants.rightStick = frc.robot.Robot.DRIV_CONTROLLER.getRightY();
        } else {
            RobotConstants.rightStick = 0;
        }

        if (frc.robot.Robot.DRIV_CONTROLLER.getLeftTriggerAxis() >= RobotConstants.deadzone) {
            RobotConstants.leftTrigger = frc.robot.Robot.DRIV_CONTROLLER.getLeftTriggerAxis();
        } else {
            RobotConstants.leftTrigger = 0;
        }

        if (frc.robot.Robot.DRIV_CONTROLLER.getRightTriggerAxis() >= RobotConstants.deadzone) {
            RobotConstants.rightTrigger = frc.robot.Robot.DRIV_CONTROLLER.getRightTriggerAxis();
        } else {
            RobotConstants.rightTrigger = 0;
        }

        // booleans, so no deadzone needed
        RobotConstants.leftBumper = frc.robot.Robot.DRIV_CONTROLLER.getLeftBumper();
        RobotConstants.rightBumper = frc.robot.Robot.DRIV_CONTROLLER.getRightBumper();
        RobotConstants.bButton = frc.robot.Robot.DRIV_CONTROLLER.getBButton();
        RobotConstants.xButton = frc.robot.Robot.DRIV_CONTROLLER.getXButton();
        RobotConstants.aButton = frc.robot.Robot.DRIV_CONTROLLER.getAButton();

    }

    public static void updateSensorValues() {
        RobotConstants.encoderPos = frc.robot.Robot.encoder.getDistance();
        RobotConstants.encoderRat = frc.robot.Robot.encoder.getRate();
    }
}
