package frc.robot.constants;

import frc.robot.Robot;

public class RobotConstants {

    // public static final double leftDrift = 1; // at least one drift should be 1,
    // both if drivebase if fine.
    // public static final double rightDrift = 1;
    public static final double autonSpeed = 0.25;
    public static final double Drivdeadzone = .03;
    public static final double Oppdeadzone = .07;

    // driving
    public static final double robotMaxSpeed = 0.5;
    public static final double slowModeMaxSpeed = 0.125;
    public static double robotAccMaxSpeed = 0.5;
    public static boolean slowMode=false;
    public static boolean turboMode=false;

    // elevator (all values in inches)
    public static final double kPoffset = 25;
    public static final double elevatorMaxHeight = 74;
    public static final double elevatorMaxRot = 76.25; //EXPERIMENTALLY DETERMINED
    public static final double elevatorMaxSpeed = 1;
    public static final double Level1 = 24 + 6;
    public static final double Level2 = 24 + 17 + 6;
    public static final double Level3 = 24 + (17 * 2) + 6;
    public static final double maxHgtSlowThrthHld=68;
    public static final double minDecelerationThreshold= 3;

    public static boolean PIDMode;
    // manipulator
    public static final double manMaxSPD=0.1;

}
