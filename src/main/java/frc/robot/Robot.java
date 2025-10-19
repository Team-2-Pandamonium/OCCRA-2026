package frc.robot;

import frc.robot.commands.UpdatePeriodic;
import frc.robot.commands.Elevator;
import frc.robot.constants.RobotConstants;

import java.lang.reflect.GenericDeclaration;
import java.sql.Driver;
import java.util.Map;

import javax.sound.sampled.Port;
import javax.swing.ButtonModel;

import org.ejml.dense.row.linsol.InvertUsingSolve_DDRM;

import com.ctre.phoenix6.hardware.CANrange;
import com.fasterxml.jackson.annotation.ObjectIdGenerators.None;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
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

  // defining the motors and channels (please change the channels when electrical
  // is finished)
  public static final PWMSparkMax intakeShort = new PWMSparkMax(0); // intake (obviously there will be more motors)
  public static final PWMSparkMax intakeLong = new PWMSparkMax(1); // intake (obviously there will be more motors)
  public static final PWMSparkMax elevatorR = new PWMSparkMax(2);
  public static final PWMSparkMax elevatorL = new PWMSparkMax(3);
  public static final PWMSparkMax right1 = new PWMSparkMax(4);
  public static final PWMSparkMax right2 = new PWMSparkMax(5);
  public static final PWMSparkMax left1 = new PWMSparkMax(6);
  public static final PWMSparkMax left2 = new PWMSparkMax(7);
  public static final Encoder encoder = new Encoder(0, 1);
  public static final CANrange elevatorHeight = new CANrange(0);
  public static final DigitalInput stg2Top = new DigitalInput(0);
  public static final DigitalInput CarrigeTop = new DigitalInput(1);
  public static final DigitalInput CarrigeBottom = new DigitalInput(2);
  // controllers
  public static final XboxController DRIV_CONTROLLER = new XboxController(0);
  public static final XboxController OPPERA_CONTROLLER = new XboxController(1);

  private Timer autonTimer = new Timer();
  // private Timer intakeTimer = new Timer();
  // private Timer robotStartTimer = new Timer();

  public ShuffleboardTab newTabKevin = Shuffleboard.getTab("KevinTabV2");
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
    elevatorR.setInverted(true);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    // // starting and reseting the timer used in auton
    // this.autonTimer.start();
    // this.autonTimer.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
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
    UpdatePeriodic.updateControllerInputs();

    // 0.6 is the speed multiplier for normal driving
    // double drivetrain_left_speed = -RobotConstants.leftStick;
    // double drivetrain_right_speed = -RobotConstants.rightStick;
    // double
    // drivetrain_max_left_speed=-1.2*(RobotConstants.leftStick*RobotConstants.leftStick);
    // double
    // drivetrain_max_right_speed=-1.2*(RobotConstants.rightStick*RobotConstants.rightStick);
    // y is up elevator
    if (DRIV_CONTROLLER.getYButton()) {
      elevatorR.set(RobotConstants.elevator_speed);
      elevatorL.set(-RobotConstants.elevator_speed);
    } else if (RobotConstants.aButton) {
      // a is down elevator
      elevatorR.set(-RobotConstants.elevator_speed);
      elevatorL.set(RobotConstants.elevator_speed);
    } else {
      elevatorR.set(0);
      elevatorL.set(0);
    }

    // b is intake
    if (RobotConstants.bButton) {
      intakeShort.set(RobotConstants.intake_speed);
      intakeLong.set(-RobotConstants.intake_speed);
    } else if (RobotConstants.xButton) {
      // x is outake
      intakeShort.set(-RobotConstants.intake_speed);
      intakeLong.set(RobotConstants.intake_speed);
    } else {
      intakeShort.set(0);
      intakeLong.set(0);
    }

    // if (RobotConstants.rightTrigger>0.75) {
    // if not rt pressed, go at normal speed
    left1.set(-RobotConstants.leftStick);
    left2.set(-RobotConstants.leftStick);
    right1.set(-RobotConstants.rightStick);
    right2.set(-RobotConstants.rightStick);
    // }

    /*
     * if (RobotConstants.rightTrigger<0.75) {
     * // when rt held, go hyperspeed
     * if (RobotConstants.leftStick<-0.05) {
     * left1.set(drivetrain_max_left_speed);
     * left2.set(drivetrain_max_left_speed);
     * } else if (RobotConstants.leftStick>0.05) {
     * left1.set(-drivetrain_max_left_speed);
     * left2.set(-drivetrain_max_left_speed);
     * } else {
     * left1.set(0);
     * left2.set(0);
     * }
     * if (RobotConstants.rightStick<-0.05) {
     * right1.set(drivetrain_max_right_speed);
     * right2.set(drivetrain_max_right_speed);
     * } else if (RobotConstants.rightStick>0.05) {
     * right1.set(-drivetrain_max_right_speed);
     * right2.set(-drivetrain_max_right_speed);
     * } else {
     * right1.set(0);
     * right2.set(0);
     * }
     */
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

}
