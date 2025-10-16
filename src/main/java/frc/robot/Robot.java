//Neel said that if the code is unsadisfactory, we can always revert it
package frc.robot;

import frc.robot.commands.MotorOutputs;
import frc.robot.commands.UpdatePeriodic;
import frc.robot.constants.RobotConstants;

import java.lang.reflect.GenericDeclaration;
import java.sql.Driver;
import java.util.Map;

import javax.sound.sampled.Port;
import javax.swing.ButtonModel;

import org.ejml.dense.row.linsol.InvertUsingSolve_DDRM;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SensorUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.InternalButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.lang.Math;
import java.lang.ModuleLayer.Controller;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/*
 * CONTROL SCHEME
 *  Driving -
 * left stick = left wheels
 * right stick = right wheels
 * right trigger = hyperspeed
 * 
 * Other functions -
 * Y button = raise elevator
 * A button = lower elevator
 * B button = intake
 * X button = outake
 */
public class Robot extends TimedRobot {

  // defining the motors and channels (please change the channels when electrical is finished)
  private final PWMSparkMax intake = new PWMSparkMax(5); // intake (obviously there will be more motors)
  private final PWMSparkMax elevator1 = new PWMSparkMax(6);
  private final PWMSparkMax elevator2 = new PWMSparkMax(2); // elevator
  private final PWMSparkMax right1 = new PWMSparkMax(4);
  private final PWMSparkMax right2 = new PWMSparkMax(1);
  private final PWMSparkMax left1 = new PWMSparkMax(3);
  private final PWMSparkMax left2 = new PWMSparkMax(0);

  public static DigitalInput beamBreak = new DigitalInput(0);
  public static XboxController controller_1 = new XboxController(0);

  private Timer autonTimer = new Timer();
  // private Timer intakeTimer = new Timer();
  // private Timer robotStartTimer = new Timer();

  public ShuffleboardTab newTabKevin = Shuffleboard.getTab("KevinTabV2");
  private GenericEntry robotAcceleration = newTabKevin.add("Robot Acceleration", .1).getEntry();
  private GenericEntry activeCruiseMode = newTabKevin.add("Cruise Mode", -111).getEntry();
  private GenericEntry cameraRequirement = newTabKevin.add("Camera Requirements", 0).getEntry();
  // private GenericEntry sensorState = newTabKevin.add("Beam break sensor",
  // false).getEntry();
  // private GenericEntry cruiseOnOff = newTabKevin.add("cruise enable",
  // RobotConstants.cruiseControl).withWidget(BuiltInWidgets.kToggleButton).getEntry();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // timer to make the loader not move on startup
    // this.robotStartTimer.start();
    // inverting one motor per gearbox to keep directionality
    right1.setInverted(true);
    right2.setInverted(true);
    intake.setInverted(true);
    elevator1.setInverted(true);
    elevator2.setInverted(true);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    // starting and reseting the timer used in auton
    this.autonTimer.start();
    this.autonTimer.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // auton code
    if (this.autonTimer.get() < 1) {
      // timer less than 1
    } else if (this.autonTimer.get() < 1.5) {
      // time less than 1.5 and bigger than 1
    } else if (this.autonTimer.get() < 2.5) {
      // time less than 2.5 and bigger than 1.5
    } else if (this.autonTimer.get() < 3.75) {
      // time less than 3.75 and bigger than 2.5
    } else {
      // after 3.75
    }
  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    // this.intakeTimer.start();

  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    System.out.println("Code is running");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // loader.set(RobotConstants.loaderPower);
    // right1.set(-0.5);
    // right2.set(-0.5);
    // left1.set(-0.5 * 0.95);
    // left2.set(-0.5 * 0.95);

  }
  double elevator_speed=0.5;
  double intake_speed=0.5;
  public void robot_controls() {
    //  0.6 is the speed multiplier for normal driving
    double drivetrain_left_speed=-(RobotConstants.leftStick*RobotConstants.leftStick);
    double drivetrain_right_speed=-(RobotConstants.rightStick*RobotConstants.rightStick);
    // double drivetrain_max_left_speed=-1.2*(RobotConstants.leftStick*RobotConstants.leftStick);
    // double drivetrain_max_right_speed=-1.2*(RobotConstants.rightStick*RobotConstants.rightStick);
    // y is up elevator
    if (controller_1.getYButton()) {
      elevator1.set(elevator_speed);
      elevator2.set(-elevator_speed);
    } else if (RobotConstants.aButton) {
    // a is down elevator
      elevator1.set(-elevator_speed);
      elevator2.set(elevator_speed);
    } else {
      elevator1.set(0);
      elevator2.set(0);
    }


    // b is intake
    if (RobotConstants.bButton) {
      intake.set(intake_speed);
    } else if (RobotConstants.xButton) {
    //x is outake
      intake.set(-intake_speed);
    } else {
      intake.set(0);
    }


    //if (RobotConstants.rightTrigger>0.75) {
    //if not rt pressed, go at normal speed
    // btw the values for leftStick and rightStick have an automatic dead zone (check RobotConstants) and drivetrain_left/right_speed automatically unfuck the left and right sticks reverse output
    left1.set(drivetrain_left_speed);
    left2.set(drivetrain_left_speed);
    right1.set(drivetrain_right_speed);
    right2.set(drivetrain_right_speed);
    //}



    /* if (RobotConstants.rightTrigger<0.75) {
    // when rt held, go hyperspeed
      if (RobotConstants.leftStick<-0.05) {
        left1.set(drivetrain_max_left_speed);
        left2.set(drivetrain_max_left_speed);
      } else if (RobotConstants.leftStick>0.05) {
        left1.set(-drivetrain_max_left_speed);
        left2.set(-drivetrain_max_left_speed);
      } else {
        left1.set(0);
        left2.set(0);
      }
      if (RobotConstants.rightStick<-0.05) {
        right1.set(drivetrain_max_right_speed);
        right2.set(drivetrain_max_right_speed);
      } else if (RobotConstants.rightStick>0.05) {
        right1.set(-drivetrain_max_right_speed);
        right2.set(-drivetrain_max_right_speed);
      } else {
        right1.set(0);
        right2.set(0);
      }
    */ 
  }
}
