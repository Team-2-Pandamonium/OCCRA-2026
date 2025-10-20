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
        //Driver
        if (Math.abs(frc.robot.Robot.DRIV_CONTROLLER.getLeftY()) >= RobotConstants.deadzone) {
            RobotConstants.DrivleftStick = frc.robot.Robot.DRIV_CONTROLLER.getLeftY();
        } else {
            RobotConstants.DrivleftStick = 0;
        }

        if (Math.abs(frc.robot.Robot.DRIV_CONTROLLER.getRightY()) >= RobotConstants.deadzone) {
            RobotConstants.DrivrightStick = frc.robot.Robot.DRIV_CONTROLLER.getRightY();
        } else {
            RobotConstants.DrivrightStick = 0;
        }

        if (frc.robot.Robot.DRIV_CONTROLLER.getLeftTriggerAxis() >= RobotConstants.deadzone) {
            RobotConstants.DrivleftTrigger = frc.robot.Robot.DRIV_CONTROLLER.getLeftTriggerAxis();
        } else {
            RobotConstants.DrivleftTrigger = 0;
        }

        if (frc.robot.Robot.DRIV_CONTROLLER.getRightTriggerAxis() >= RobotConstants.deadzone) {
            RobotConstants.DrivrightTrigger = frc.robot.Robot.DRIV_CONTROLLER.getRightTriggerAxis();
        } else {
            RobotConstants.DrivrightTrigger = 0;
        }

        // booleans, so no deadzone needed, driver
        RobotConstants.DrivleftBumper = frc.robot.Robot.DRIV_CONTROLLER.getLeftBumper();
        RobotConstants.DrivrightBumper = frc.robot.Robot.DRIV_CONTROLLER.getRightBumper();
        RobotConstants.DrivbButton = frc.robot.Robot.DRIV_CONTROLLER.getBButton();
        RobotConstants.DrivxButton = frc.robot.Robot.DRIV_CONTROLLER.getXButton();
        RobotConstants.DrivaButton = frc.robot.Robot.DRIV_CONTROLLER.getAButton();
        RobotConstants.DrivyButton = frc.robot.Robot.DRIV_CONTROLLER.getYButton();
        // Opperator
        if (Math.abs(frc.robot.Robot.OPPERA_CONTROLLER.getLeftY()) >= RobotConstants.deadzone) {
            RobotConstants.OpperaleftStick = frc.robot.Robot.OPPERA_CONTROLLER.getLeftY();
        } else {
            
            RobotConstants.OpperaleftStick = 0;
        }

        if (Math.abs(frc.robot.Robot.OPPERA_CONTROLLER.getRightY()) >= RobotConstants.deadzone) {
            RobotConstants.OpperarightStick = frc.robot.Robot.OPPERA_CONTROLLER.getRightY();
        } else {
            RobotConstants.OpperarightStick = 0;
        }

        if (frc.robot.Robot.OPPERA_CONTROLLER.getLeftTriggerAxis() >= RobotConstants.deadzone) {
            RobotConstants.OpperaleftTrigger = frc.robot.Robot.OPPERA_CONTROLLER.getLeftTriggerAxis();
        } else {
            RobotConstants.OpperaleftTrigger = 0;
        }

        if (frc.robot.Robot.OPPERA_CONTROLLER.getRightTriggerAxis() >= RobotConstants.deadzone) {
            RobotConstants.OpperarightTrigger = frc.robot.Robot.OPPERA_CONTROLLER.getRightTriggerAxis();
        } else {
            RobotConstants.OpperarightTrigger = 0;
        }
        
        RobotConstants.OpperaleftBumper = frc.robot.Robot.OPPERA_CONTROLLER.getLeftBumper();
        RobotConstants.OpperarightBumper = frc.robot.Robot.OPPERA_CONTROLLER.getRightBumper();
        RobotConstants.OpperabButton = frc.robot.Robot.OPPERA_CONTROLLER.getBButton();
        RobotConstants.OpperaxButton = frc.robot.Robot.OPPERA_CONTROLLER.getXButton();
        RobotConstants.OpperaaButton = frc.robot.Robot.OPPERA_CONTROLLER.getAButton();
        RobotConstants.OpperayButton = frc.robot.Robot.OPPERA_CONTROLLER.getYButton();
    }

    public static void updateSensorValues() {
        RobotConstants.encoderPos = frc.robot.Robot.encoder.getDistance();
        RobotConstants.encoderRat = frc.robot.Robot.encoder.getRate();
    }
}
