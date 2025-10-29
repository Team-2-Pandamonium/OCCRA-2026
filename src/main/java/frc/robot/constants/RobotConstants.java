package frc.robot.constants;

public class RobotConstants {

    // public static final double leftDrift = 1; // at least one drift should be 1,
    // both if drivebase if fine.
    // public static final double rightDrift = 1;
    public static final double autonSpeed = .5;
    public static final double deadzone = .05;

    // driving
    public static final double robotMaxSpeed = 0.5;
    public static double right1Enc;
    public static double right2Enc;
    public static double left1Enc;
    public static double left2Enc;

    // elevator (all values in inches)
    public static final double kPoffset=1;
    public static final double GrdOff=(8+9/16);
    public static final double elevatorMaxHeight = 60;
    public static final double elevatorMaxRot = 76.25; //EXPERIMENTALLY DETERMINED
    public static final double elevatorMaxSpeed = 0.75;
    public static double elevatorHeight; //height in Inches
    public static double elevatorRotHeight; //height in rotations
    public static final double Level1 = 24;
    public static final double Level1GrdOff = 24 - GrdOff;
    public static final double Level2 = 24 + 17;
    public static final double Level2GrdOff = 24 + 17 - GrdOff;
    public static final double Level3 = 24 + 17 * 2;
    public static final double Level3GrdOff = 24 + 17 * 2 - GrdOff;
    public static final double humanPlayer = 35;
    public static final double humanPlayerGrdOff = 35 - GrdOff;
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
    // inputs, operator
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
    public static boolean OpperaDPadUp;
    public static boolean OpperaDPadUpRight;
    public static boolean OpperaDPadRight;
    public static boolean OpperaDPadDownRight;
    public static boolean OpperaDPadDown;
    public static boolean OpperaDPadDownLeft;
    public static boolean OpperaDPadLeft;
    public static boolean OpperaDPadUpLeft;
}
