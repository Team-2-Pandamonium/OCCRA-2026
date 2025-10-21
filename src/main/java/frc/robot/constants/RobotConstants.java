package frc.robot.constants;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class RobotConstants {

    // public static final double leftDrift = 1; // at least one drift should be 1,
    // both if drivebase if fine.
    // public static final double rightDrift = 1;
    public static final double autonSpeed = .5;
    public static final double deadzone = .01;

    // driving
    public static final double robotMaxSpeed = 1;

    // elevator (all values in inches)
    public static final double elevatorMaxSpeed = 1;
    public static final double intake_speed = 0.5;
    public static double elevatorHeight;
    public static final double Level1 = 24;
    public static final double Level2 = 24 + 17;
    public static final double Level3 = 24 + 17 * 2;
    public static final double humanPlayer = 35;
    public static double encoderPos;
    public static double encoderRat;
    public static boolean carrigeTop;
    public static boolean stg2Top;
    public static boolean carrigeBot;
    // manipulaor

    // inputs, driver
    public static double DrivrightTrigger;
    public static double DrivleftStick;
    public static double DrivrightStick;
    public static Boolean DrivleftBumper;
    public static Boolean DrivrightBumper;
    public static double DrivleftTrigger;
    public static boolean DrivbButton;
    public static boolean DrivxButton;
    public static boolean DrivaButton;
    public static boolean DrivyButton;
    // operator
    public static double OpperarightTrigger;
    public static double OpperaleftStick;
    public static double OpperarightStick;
    public static Boolean OpperaleftBumper;
    public static Boolean OpperarightBumper;
    public static double OpperaleftTrigger;
    public static boolean OpperabButton;
    public static boolean OpperaxButton;
    public static boolean OpperaaButton;
    public static boolean OpperayButton;

}
