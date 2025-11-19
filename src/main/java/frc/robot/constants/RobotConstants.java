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
    public static final double elevatorMaxSpeed = 1;
    public static double elevatorRotHeight; //height in rotations
    public static final double Level1 = 24 + 6;
    public static final double Level2 = 24 + 17 + 6;
    public static final double Level3 = 24 + (17 * 2) + 6;
    public static final double maxHeightforSlowThreshold=72;
    public static final double minHeightforSlowThreshold=3;
    public static final double minHeightforDecelerationThreshold=5;
    public static boolean carrigeTop;
    public static boolean stg2Top;
    public static boolean carrigeBot;
    public static double elevatorOutput;
    public static boolean topEndstop;
    public static boolean bottEndstop;
    public static boolean PIDMode;
    public static int level;
    // manipulator
    public static final double manMaxSPD=0.1;
    public static double manLeftOutput = 0.5;
    public static double manRightOutput = 0.25;
    public static boolean spazMode = false;


    // drive
    public static double rightOutput;
    public static double leftOutput;
    public static double robotAngle;

    // inputs, driver
    public static double DrivrightTrigger;
    public static double DrivleftStick;
    public static double DrivrightStick;
    public static Boolean DrivleftBumper=false;
    public static Boolean DrivrightBumper=false;
    public static double DrivleftTrigger=0;
    public static boolean DrivbButton=false;
    public static boolean DrivxButton=false;
    public static boolean DrivaButton=false;
    public static boolean DrivyButton=false;
    // inputs, operator
    public static double OpperarightTrigger=0;
    public static double OpperaleftStick=0;
    public static double OpperarightStick=0;
    public static Boolean OpperaleftBumper=false;
    public static Boolean OpperarightBumper=false;
    public static double OpperaleftTrigger=0;
    public static boolean OpperabButton=false;
    public static boolean OpperaxButton=false;
    public static boolean OpperaaButton=false;
    public static boolean OpperayButton=false;
    public static boolean OpperaDPadUp=false;
    public static boolean OpperaDPadUpRight=false;
    public static boolean OpperaDPadRight=false;
    public static boolean OpperaDPadDownRight=false;
    public static boolean OpperaDPadDown=false;
    public static boolean OpperaDPadDownLeft=false;
    public static boolean OpperaDPadLeft=false;
    public static boolean OpperaDPadUpLeft=false;
}
