package frc.robot.commands;

import frc.robot.constants.RobotConstants;

public class MotorOutputs {
    /**
     * finds the actual value for the motors based on the current and desired
     * power levels
     * 
     * @param target
     * @param output
     * @param accelerationRate
     * @return <b>actual output
     */
    public static double findOutput(double target, double output, double accelerationRate) {

        if ((target - output) > accelerationRate) {
            output += accelerationRate;
            // target > output
        } else if ((output - target) > accelerationRate) {
            output -= accelerationRate;
            // output > target
        } else {
            output = target;
        }

        return output;
    }

    /**
     * makes triggers affect driving
     * <p>
     * returns an
     * 
     * 
     * @param leftStick
     * @param leftTrigger
     * @param rightStick
     * @param rightTrigger
     * @param FormerOutputLeft
     * @param FormerOutputRight
     * @param accelerationRate
     * @return <b>array of doubles (rightCruiseOutput, leftCruiseOutput, cruiseMode)
     */
    public static double[] superCruiseOutput(double leftStick, double leftTrigger, double rightStick,
            double rightTrigger, double FormerOutputLeft, double FormerOutputRight, double accelerationRate) {
        double rightCruiseOutput = 0;
        double leftCruiseOutput = 0;
        double cruiseMode = 999;

        if (rightStick == 0 && leftStick == 0 && rightTrigger == 0 && leftTrigger == 0) { // nothing is in use
            rightCruiseOutput = 0;
            leftCruiseOutput = 0;
            cruiseMode = 1;
            // no input at all
        } else if (rightStick == 0 && leftStick == 0 && rightTrigger == 0) { // only
            // the left trigger is in use
            rightCruiseOutput = 0;
            leftCruiseOutput = leftTrigger * -.3;
            cruiseMode = 2;
        } else if (rightStick == 0 && leftStick == 0 && leftTrigger == 0) { // only
            // the right trigger is in use
            leftCruiseOutput = 0;
            rightCruiseOutput = rightTrigger * -.3;
            cruiseMode = 3;
        } else if (leftTrigger == 0 && rightTrigger == 0 && ((Math.abs(rightStick) > 0) || (Math.abs(leftStick) > 0))) {
            // only the sticks in use (or just one)
            leftCruiseOutput = leftStick;
            rightCruiseOutput = rightStick;
            cruiseMode = 4;
        } else if (leftTrigger > 0 && rightTrigger > 0) {
            // both triggers in use
            leftCruiseOutput = leftStick * .3;
            rightCruiseOutput = rightStick * .3;
            cruiseMode = 5;
        } else {
            // catch statment to default to normal control scheme
            leftCruiseOutput = leftStick;
            rightCruiseOutput = rightStick;
            cruiseMode = 6;
        }

        if (RobotConstants.cruiseControl == true) { // on
            rightCruiseOutput = findOutput(rightCruiseOutput, FormerOutputRight, accelerationRate);
            leftCruiseOutput = findOutput(leftCruiseOutput, FormerOutputLeft, accelerationRate);
        } else if (RobotConstants.cruiseControl == false) { // off
            rightCruiseOutput = findOutput(rightStick, FormerOutputRight, accelerationRate);
            leftCruiseOutput = findOutput(leftStick, FormerOutputLeft, accelerationRate);

        }

        double[] cruiseOutput = { rightCruiseOutput, leftCruiseOutput, cruiseMode };
        return cruiseOutput;
    }

}
