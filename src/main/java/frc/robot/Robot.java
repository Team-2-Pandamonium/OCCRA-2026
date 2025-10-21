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

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANrange;
import com.fasterxml.jackson.annotation.ObjectIdGenerators.None;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase;

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
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
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

  //motors
  public static final SparkMax manShort = new SparkMax(0, MotorType.kBrushless);
  public static final SparkMax manLong = new SparkMax(1, MotorType.kBrushless);
  public static final SparkMax elevatorR = new SparkMax(21, MotorType.kBrushless);
  public static final SparkMax elevatorL = new SparkMax(22, MotorType.kBrushless);
  public static final SparkMax right1 = new SparkMax(11, MotorType.kBrushless);
  public static final SparkMax right2 = new SparkMax(12, MotorType.kBrushless);
  public static final SparkMax left1 = new SparkMax(13, MotorType.kBrushless);
  public static final SparkMax left2 = new SparkMax(14, MotorType.kBrushless);
  //sensors
  public static final Encoder encoder = new Encoder(4, 5);
  public static final CANrange elevatorHeight = new CANrange(3);
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
  public GenericEntry cameraRequirement = newTabKevin.add("Camera Requirements", 0).getEntry();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // SparkMaxConfig config = new SparkMaxConfig();
    // config.idleMode(IdleMode.kBrake).smartCurrentLimit(40).disableFollowerMode().inverted(false);
    // SparkMaxConfig followerConfig = new SparkMaxConfig();
    // followerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(false);

    // right1.configure(config, null, null);
    // followerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(false).follow(right1);
    // right2.configure(followerConfig, null, null);

    // config.inverted(true);
    // left1.configure(config, null, null);
    // followerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(true).follow(left1);
    // left2.configure(followerConfig, null, null);

    // config.inverted(false);
    // elevatorR.configure(config, null, null);
    // followerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(false).follow(elevatorR, true);
    // elevatorL.configure(followerConfig, null, null);

    // manLong.configure(config, null, null);
    // manShort.configure(config, null, null);

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

    // if (RobotConstants.DrivyButton) {
    // elevatorR.set(RobotConstants.elevator_speed);
    // elevatorL.set(RobotConstants.elevator_speed);
    // } else if (RobotConstants.DrivaButton) {
    // // a is down elevator
    // elevatorR.set(RobotConstants.elevator_speed);
    // elevatorL.set(RobotConstants.elevator_speed);
    // } else {
    // elevatorR.set(0);
    // elevatorL.set(0);
    // }

    // // b is intake
    // if (RobotConstants.DrivbButton) {
    // manShort.set(RobotConstants.intake_speed);
    // manLong.set(-RobotConstants.intake_speed);
    // } else if (RobotConstants.DrivxButton) {
    // // x is outake
    // manShort.set(-RobotConstants.intake_speed);
    // manLong.set(RobotConstants.intake_speed);
    // } else {
    // manShort.set(0);
    // manLong.set(0);
    // }
    // // sets the speed of the elevator motors based on what the operator inputs
    // if (RobotConstants.OpperaaButton && (RobotConstants.OpperarightTrigger > 0 ||
    // RobotConstants.OpperarightBumper)) { // lvl1r
    // elevatorR.set(Elevator.dumbCalcMotSpd(4, RobotConstants.elevatorHeight));
    // } else if (RobotConstants.OpperabButton
    // && (RobotConstants.OpperarightTrigger > 0 ||
    // RobotConstants.OpperarightBumper)) { // lvl2 r
    // elevatorR.set(Elevator.dumbCalcMotSpd(5, RobotConstants.elevatorHeight));
    // } else if (RobotConstants.OpperaxButton
    // && (RobotConstants.OpperarightTrigger > 0 ||
    // RobotConstants.OpperarightBumper)) { // lvl3 r
    // elevatorR.set(Elevator.dumbCalcMotSpd(6, RobotConstants.elevatorHeight));
    // } else if (RobotConstants.OpperaaButton) { // lvl1
    // elevatorR.set(Elevator.dumbCalcMotSpd(1, RobotConstants.elevatorHeight));
    // } else if (RobotConstants.OpperabButton) { // lvl2
    // elevatorR.set(Elevator.dumbCalcMotSpd(2, RobotConstants.elevatorHeight));
    // } else if (RobotConstants.OpperaxButton) { // lvl3
    // elevatorR.set(Elevator.dumbCalcMotSpd(3, RobotConstants.elevatorHeight));
    // } else if (RobotConstants.OpperayButton) { // hp
    // elevatorR.set(Elevator.dumbCalcMotSpd(7, RobotConstants.elevatorHeight));
    // }


    left1.set(-RobotConstants.DrivleftStick * RobotConstants.robotMaxSpeed);
    right1.set(RobotConstants.DrivrightStick * RobotConstants.robotMaxSpeed);
    left2.set(-RobotConstants.DrivleftStick * RobotConstants.robotMaxSpeed);
    right2.set(RobotConstants.DrivrightStick * RobotConstants.robotMaxSpeed);
    elevatorL.set(RobotConstants.OpperaleftStick*RobotConstants.elevatorMaxSpeed);
    elevatorR.set(-RobotConstants.OpperaleftStick*RobotConstants.elevatorMaxSpeed);

  }

  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {
    System.out.println("Code is running");
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {


  }

}
