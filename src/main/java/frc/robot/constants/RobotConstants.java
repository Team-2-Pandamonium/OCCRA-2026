package frc.robot.constants;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class RobotConstants {

    // public static final double leftDrift = 1; // at least one drift should be 1,
    // both if drivebase if fine.
    // public static final double rightDrift = 1;
    public static final double autonSpeed = .5;
    public static final double deadzone = .05;

    // driving
    public static final double robotSpeedSave = 1;
    public static final double accelerationRate = .2;
    public static double leftOutput = 0;
    public static double rightOutput = 0;
    public static double leftTarget = 0;
    public static double rightTarget = 0;
    public static double robotSpeed = 0.5;
    public static boolean cruiseControl = false; // fasle=off true=on

    // shooting/loading
    public static final double loaderPower = 0.4;
    public static double launcherLimit = 0.8;
    public static double loaderOutput = 0;
    public static boolean shooterTogglePressed = false;
    public static boolean shooterOutput = false;
    public static boolean hasBall = false;
    // 0.5 not high enough
    // 0.8 most consistant

    // not really constansts but whatever
    // inputs
    public static double rightTrigger;
    public static double leftStick;{if (Math.abs(leftStick) <= .05) {leftStick=0;} else{leftStick=leftStick;}}
    public static double rightStick;{if (Math.abs(rightStick) <= .05) {rightStick=0;} else{rightStick=rightStick;}}
    public static Boolean leftBumper;
    public static Boolean rightBumper;
    public static double leftTrigger;
    public static boolean bButton;
    public static boolean xButton;
    public static boolean aButton;
    public static boolean beamSensor; // true = connects false = somthing blocking

    // loader code (prev version)
    // public static double intakeContactRemovalTime = 0.0;
    // public static Boolean intakeWaitVar = false;
    // public static Boolean intakeRun = false;
    // public static double intakeRunTime = 0.0;
    // public static Boolean loaderControllerDisconnect = false;

    // public static double rollbackFactor = .25;
    // public static double rollbackMax = 2;

}
