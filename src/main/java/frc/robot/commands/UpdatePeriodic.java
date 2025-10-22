package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.PIDVar;

import java.lang.reflect.Type;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;

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

        RobotConstants.OpperaDPadUp = false;
        RobotConstants.OpperaDPadUpRight = false;
        RobotConstants.OpperaDPadRight = false;
        RobotConstants.OpperaDPadDownRight = false;
        RobotConstants.OpperaDPadDown = false;
        RobotConstants.OpperaDPadDownLeft = false;
        RobotConstants.OpperaDPadLeft = false;
        RobotConstants.OpperaDPadUpLeft = false;

        int OpperaPOV = Robot.OPPERA_CONTROLLER.getPOV();
        switch (OpperaPOV) {
            case 0: // up
                RobotConstants.OpperaDPadUp = true;
                break;
            case 45: // up-right
                RobotConstants.OpperaDPadUpRight = true;

                break;
            case 90: // right
                RobotConstants.OpperaDPadRight = true;

                break;
            case 135: // down-right
                RobotConstants.OpperaDPadDownRight = true;

                break;
            case 180: // down
                RobotConstants.OpperaDPadDown = true;

                break;
            case 225: // down-left
                RobotConstants.OpperaDPadDownLeft = true;

                break;
            case 270: // left
                RobotConstants.OpperaDPadLeft = true;

                break;
            case 315: // up-left
                RobotConstants.OpperaDPadUpLeft = true;
                break;

            case -1: // Nothing pressed

                break;
            default:
                System.err.println("Error somehow you pressed a button on the DPad that doesnt exist");
                break;
        }
    }

    public static void updateSensorValues() {
        RobotConstants.elevatorRotHeight = frc.robot.Robot.elevatorEnc.getDistance();
        RobotConstants.carrigeBot=frc.robot.Robot.CarrigeBottom.get();
        RobotConstants.carrigeTop=frc.robot.Robot.CarrigeTop.get();
        RobotConstants.stg2Top=frc.robot.Robot.stg2Top.get();
        RobotConstants.elevatorHeight = Elevator.RottoIn(RobotConstants.elevatorRotHeight);
        if (RobotConstants.carrigeBot && RobotConstants.elevatorHeight != 0) {
            System.err.println(
                    "ERROR: elevator encoder not reading 0 when at bottom, rezeroed " + RobotConstants.elevatorHeight);
            RobotConstants.elevatorHeight = 0;
        }
    }
}
