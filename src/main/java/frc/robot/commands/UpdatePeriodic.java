package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.constants.RobotConstants;

public class UpdatePeriodic {
    /**
     * Gets the value from the controllers and updates it in the code
     */
    public static void updateControllerInputs() {
        // if any of these are below the deadzone, it equals zero. abs so the controller
        // can go negative
        //Driver
        if (Math.abs(frc.robot.Robot.DRIV_CONTROLLER.getLeftY()) >= RobotConstants.Drivdeadzone) {
            RobotConstants.DrivleftStick = frc.robot.Robot.DRIV_CONTROLLER.getLeftY();
        } else {
            RobotConstants.DrivleftStick = 0;
        }

        if (Math.abs(frc.robot.Robot.DRIV_CONTROLLER.getRightY()) >= RobotConstants.Drivdeadzone) {
            RobotConstants.DrivrightStick = frc.robot.Robot.DRIV_CONTROLLER.getRightY();
        } else {
            RobotConstants.DrivrightStick = 0;
        }

        if (frc.robot.Robot.DRIV_CONTROLLER.getRawAxis(3) >= RobotConstants.Drivdeadzone) {
            RobotConstants.DrivleftTrigger = frc.robot.Robot.DRIV_CONTROLLER.getRawAxis(3);
        } else {
            RobotConstants.DrivleftTrigger = 0;
        }

        if (frc.robot.Robot.DRIV_CONTROLLER.getRawAxis(4) >= RobotConstants.Drivdeadzone) {
            RobotConstants.DrivrightTrigger = frc.robot.Robot.DRIV_CONTROLLER.getRawAxis(4);
        } else {
            RobotConstants.DrivrightTrigger = 0;
        }

        // booleans, so no deadzone needed, driver
        RobotConstants.DrivleftBumper = frc.robot.Robot.DRIV_CONTROLLER.getRawButton(5);
        RobotConstants.DrivrightBumper = frc.robot.Robot.DRIV_CONTROLLER.getRawButton(6);

        // Opperator
        if (Math.abs(frc.robot.Robot.OPPERA_CONTROLLER.getLeftY()) >= RobotConstants.Oppdeadzone) {
            RobotConstants.OpperaleftStick = frc.robot.Robot.OPPERA_CONTROLLER.getLeftY();
        } else {
            
            RobotConstants.OpperaleftStick = 0;
        }

        if (Math.abs(frc.robot.Robot.OPPERA_CONTROLLER.getRightY()) >= RobotConstants.Oppdeadzone) {
            RobotConstants.OpperarightStick = frc.robot.Robot.OPPERA_CONTROLLER.getRightY();
        } else {
            RobotConstants.OpperarightStick = 0;
        }

        if (frc.robot.Robot.OPPERA_CONTROLLER.getLeftTriggerAxis() >= RobotConstants.Oppdeadzone) {
            RobotConstants.OpperaleftTrigger = frc.robot.Robot.OPPERA_CONTROLLER.getLeftTriggerAxis();
        } else {
            RobotConstants.OpperaleftTrigger = 0;
        }

        if (frc.robot.Robot.OPPERA_CONTROLLER.getRightTriggerAxis() >= RobotConstants.Oppdeadzone) {
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
        // System.out.println(RobotConstants.OpperaDPadUp);
    }

    public static void updateSensorValues() {
        RobotConstants.elevatorRotHeight = frc.robot.Robot.elevatorEnc.getPosition();
        RobotConstants.carrigeBot=frc.robot.Robot.CarrigeBottom.get();
        RobotConstants.carrigeTop=frc.robot.Robot.CarrigeTop.get();
        RobotConstants.stg2Top=frc.robot.Robot.stg2Top.get();
        // RobotConstants.elevatorHeight = Elevator.RottoIn(RobotConstants.elevatorRotHeight);

        if (RobotConstants.carrigeBot == false) {
            RobotConstants.bottEndstop = true;
            // System.out.println("at bottom");
        }else{
            RobotConstants.bottEndstop=false;
            // System.out.println("not at bottom");
        }

        if ((RobotConstants.stg2Top == false) && (RobotConstants.carrigeTop == false)) {
            RobotConstants.topEndstop = true;
            // System.out.println("at top");
        } else {
            RobotConstants.topEndstop = false;
            // System.out.println("not at top");
        }

    }
}
