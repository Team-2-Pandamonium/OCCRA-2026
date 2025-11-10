package frc.robot.constants;

public class RobotConstants {

    // public static final double leftDrift = 1; // at least one drift should be 1,
    // both if drivebase if fine.
    // public static final double rightDrift = 1;
    public static final double autonSpeed = 0.25;
    public static final double Drivdeadzone = .03;
    public static final double Oppdeadzone = .07;

    // driving
    public static final double robotMaxSpeed = 0.5;
    public static double slowModeMaxSpeed = 0.125;
    public static double robotAccMaxSpeed = 0.5;
    public static boolean slowMode=false;
    public static boolean turboMode=false;
    public static double right1Enc;
    public static double right2Enc;
    public static double left1Enc;
    public static double left2Enc;

    // elevator (all values in inches)
    public static final double kPoffset = 25;
    public static final double elevatorMaxHeight = 60;
    public static final double elevatorMaxRot = 76.25; //EXPERIMENTALLY DETERMINED
    public static final double elevatorMaxSpeed = 0.75;
    public static double elevatorRotHeight; //height in rotations
    public static final double Level1 = 24 + 6;
    public static final double Level2 = 24 + 17 + 6;
    public static final double Level3 = 24 + (17 * 2) + 6;
    public static final double maxHgtSlowThrthHld=72;
    public static boolean carrigeTop;
    public static boolean stg2Top;
    public static boolean carrigeBot;
    public static double elevatorOutput;
    public static boolean topEndstop;
    public static boolean bottEndstop;
    // manipulator
    public static final double manMaxSPD=0.1;
    public static double manLeftOutput;
    public static double manRightOutput;

    // drive
    public static double rightOutput;
    public static double leftOutput;
    public static double robotAngle;

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
