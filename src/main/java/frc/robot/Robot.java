package frc.robot;

import frc.robot.commands.UpdatePeriodic;
import frc.robot.commands.Elevator;
import frc.robot.constants.PIDVar;
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
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;

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
import edu.wpi.first.math.controller.PIDController;
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

  // Built in encoders (Un-needed unless we want their values for some weird
  // reason)
  // public static final RelativeEncoder manShortEnc = manShort.getEncoder();
  // public static final RelativeEncoder manLongEnc = manLong.getEncoder();
  // public static final RelativeEncoder elevatorREnc = elevatorR.getEncoder();
  // public static final RelativeEncoder elevatorLEnc = elevatorL.getEncoder();
  // public static final RelativeEncoder right1Enc = right1.getEncoder();
  // public static final RelativeEncoder right2Enc = right2.getEncoder();
  // public static final RelativeEncoder left1Enc = left1.getEncoder();
  // public static final RelativeEncoder left2Enc = left2.getEncoder();
  // REV PID loop
  public static final SparkClosedLoopController elevatorRREV = elevatorR.getClosedLoopController();
  public static final SparkClosedLoopController elevatorLREV = elevatorL.getClosedLoopController();
  //sensors
  public static final Encoder elevatorEnc = new Encoder(0, 1);
  // public static final Encoder elevatorEncL = new Encoder(2, 3);
  // public static final CANrange elevatorHeight = new CANrange(3);
  public static final DigitalInput stg2Top = new DigitalInput(4);
  public static final DigitalInput CarrigeTop = new DigitalInput(5);
  public static final DigitalInput CarrigeBottom = new DigitalInput(6);


  // controllers
  public static final XboxController DRIV_CONTROLLER = new XboxController(0);
  public static final XboxController OPPERA_CONTROLLER = new XboxController(1);


  // shuffleboard
  public ShuffleboardTab newTabKevin = Shuffleboard.getTab("KevinTabV2");
  public GenericEntry cameraRequirement = newTabKevin.add("Camera Requirements", 0).getEntry();
  public GenericEntry elevatorheight = newTabKevin.add("Elevator Height: ", RobotConstants.elevatorHeight).getEntry();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    SparkMaxConfig configR1 = new SparkMaxConfig();
    configR1.idleMode(IdleMode.kBrake).smartCurrentLimit(40).disableFollowerMode().inverted(true).closedLoop
        .velocityFF(0)
        .p(PIDVar.right1P, ClosedLoopSlot.kSlot0)
        .i(PIDVar.right1I, ClosedLoopSlot.kSlot0)
        .d(PIDVar.right1D, ClosedLoopSlot.kSlot0);
    SparkMaxConfig configR2 = new SparkMaxConfig();
    configR2.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(true).follow(right1).closedLoop        
        .velocityFF(0)
        .p(PIDVar.right2P, ClosedLoopSlot.kSlot0)
        .i(PIDVar.right2I, ClosedLoopSlot.kSlot0)
        .d(PIDVar.right2D, ClosedLoopSlot.kSlot0);
    right1.configure(configR1, null, null);
    right2.configure(configR2, null, null);

    SparkMaxConfig configL1 = new SparkMaxConfig();
    configL1.idleMode(IdleMode.kBrake).smartCurrentLimit(40).disableFollowerMode().inverted(false).closedLoop
        .velocityFF(0)
        .p(PIDVar.left1P, ClosedLoopSlot.kSlot0)
        .i(PIDVar.left1I, ClosedLoopSlot.kSlot0)
        .d(PIDVar.left1D, ClosedLoopSlot.kSlot0);
    SparkMaxConfig configL2 = new SparkMaxConfig();
    configL2.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(false).follow(left1).closedLoop
        .velocityFF(0)
        .p(PIDVar.left2P, ClosedLoopSlot.kSlot0)
        .i(PIDVar.left2I, ClosedLoopSlot.kSlot0)
        .d(PIDVar.left2D, ClosedLoopSlot.kSlot0);
    left1.configure(configL1, null, null);
    left2.configure(configL2, null, null);

    SparkMaxConfig configEleR = new SparkMaxConfig();
    configEleR.idleMode(IdleMode.kBrake).smartCurrentLimit(40).disableFollowerMode().inverted(true).closedLoop
        .velocityFF(0)
        .p(PIDVar.elevatorRP, ClosedLoopSlot.kSlot0)
        .i(PIDVar.elevatorRI, ClosedLoopSlot.kSlot0)
        .d(PIDVar.elevatorRD, ClosedLoopSlot.kSlot0);
    SparkMaxConfig configEleL = new SparkMaxConfig();
    configEleL.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(true).follow(elevatorR, true).closedLoop
        .velocityFF(0)
        .p(PIDVar.elevatorLP, ClosedLoopSlot.kSlot0)
        .i(PIDVar.elevatorLI, ClosedLoopSlot.kSlot0)
        .d(PIDVar.elevatorLD, ClosedLoopSlot.kSlot0);
    elevatorR.configure(configEleR, null, null);
    elevatorL.configure(configEleL, null, null);
    SparkMaxConfig configManShort = new SparkMaxConfig();
    configManShort.idleMode(IdleMode.kBrake).smartCurrentLimit(40).disableFollowerMode().inverted(true)/*
                                                                                                        * .closedLoop
                                                                                                        * .velocityFF(0)
                                                                                                        * .p(PIDVar.
                                                                                                        * manShortP,
                                                                                                        * ClosedLoopSlot
                                                                                                        * .kSlot0)
                                                                                                        * .i(PIDVar.
                                                                                                        * manShortI,
                                                                                                        * ClosedLoopSlot
                                                                                                        * .kSlot0)
                                                                                                        * .d(PIDVar.
                                                                                                        * manShortD,
                                                                                                        * ClosedLoopSlot
                                                                                                        * .kSlot0)
                                                                                                        */;
    SparkMaxConfig configManLong = new SparkMaxConfig();
    configManLong.idleMode(IdleMode.kBrake).smartCurrentLimit(40).disableFollowerMode().inverted(true)/*
                                                                                                       * .closedLoop
                                                                                                       * .velocityFF(0)
                                                                                                       * .p(PIDVar.
                                                                                                       * manLongP,
                                                                                                       * ClosedLoopSlot.
                                                                                                       * kSlot0)
                                                                                                       * .i(PIDVar.
                                                                                                       * manLongI,
                                                                                                       * ClosedLoopSlot.
                                                                                                       * kSlot0)
                                                                                                       * .d(PIDVar.
                                                                                                       * manLongD,
                                                                                                       * ClosedLoopSlot.
                                                                                                       * kSlot0)
                                                                                                       */;
    manShort.configure(configManShort, null, null);
    manLong.configure(configManLong, null, null);
    elevatorEnc.reset();
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
    // System.out.println("WARNING: RESETING ELEVATOR 0");
    // Elevator.reset0();

  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    UpdatePeriodic.updateControllerInputs();
    UpdatePeriodic.updateSensorValues();
    newTabKevin.add("Elevator Height ", RobotConstants.elevatorHeight);
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

    RobotConstants.elevatorHeight = Elevator.RottoIn(elevatorEnc.getDistance());
    // // sets the speed of the elevator motors based on what the operator inputs
    if (!(RobotConstants.OpperaDPadDown || RobotConstants.OpperaDPadDownRight || RobotConstants.OpperaDPadUp
        || RobotConstants.OpperaDPadUpRight || RobotConstants.OpperaDPadRight)) {

      if (RobotConstants.OpperaaButton && !(RobotConstants.OpperarightBumper)) { // lvl1
        elevatorRREV.setReference(Elevator.CalcRot(1, RobotConstants.elevatorHeight), ControlType.kPosition);
      } else if (RobotConstants.OpperabButton &&
          !RobotConstants.OpperarightBumper) { // lvl2
        elevatorRREV.setReference(Elevator.CalcRot(2, RobotConstants.elevatorHeight), ControlType.kPosition);
      } else if (RobotConstants.OpperaxButton &&
          !RobotConstants.OpperarightBumper) { // lvl3
        elevatorRREV.setReference(Elevator.CalcRot(3, RobotConstants.elevatorHeight), ControlType.kPosition);
      } else if (RobotConstants.OpperayButton &&
          !RobotConstants.OpperarightBumper) { // hp
        elevatorRREV.setReference(Elevator.CalcRot(7, RobotConstants.elevatorHeight), ControlType.kPosition);
      } else if (RobotConstants.OpperaaButton &&
          RobotConstants.OpperarightBumper) { // lvl1r
        elevatorRREV.setReference(Elevator.CalcRot(4, RobotConstants.elevatorHeight), ControlType.kPosition);
      } else if (RobotConstants.OpperabButton
          && RobotConstants.OpperarightBumper) { // lvl2 r
        elevatorRREV.setReference(Elevator.CalcRot(5, RobotConstants.elevatorHeight), ControlType.kPosition);
      } else if (RobotConstants.OpperaxButton
          && RobotConstants.OpperarightBumper) { // lvl3 r
        elevatorRREV.setReference(Elevator.CalcRot(6, RobotConstants.elevatorHeight), ControlType.kPosition);
      } else if (RobotConstants.OpperaleftBumper) {
        elevatorRREV.setReference(Elevator.CalcRot(0, RobotConstants.elevatorHeight), ControlType.kPosition);
      }
    } else {
      if (RobotConstants.OpperaDPadUp && RobotConstants.elevatorHeight < RobotConstants.elevatorMaxHeight) {
        elevatorR.set(0.5);
      } else if (RobotConstants.OpperaDPadUpRight && RobotConstants.elevatorHeight < RobotConstants.elevatorMaxHeight) {
        elevatorR.set(0.2);
      } else if (RobotConstants.OpperaDPadDown && RobotConstants.elevatorHeight > 0) {
        elevatorR.set(-0.5);
      } else if (RobotConstants.OpperaDPadDownRight && RobotConstants.elevatorHeight > 0) {
        elevatorR.set(-0.2);
    }
  }
  if (RobotConstants.OpperarightTrigger > 0) { // intake
    manLong.set(RobotConstants.OpperarightTrigger);
    manLong.set(RobotConstants.OpperarightTrigger);
  } else if (RobotConstants.OpperaleftTrigger > 0) { // outtake
    manLong.set(-RobotConstants.OpperaleftTrigger);
    manLong.set(-RobotConstants.OpperaleftTrigger);
  } else {
    manLong.set(RobotConstants.OpperaleftStick);
    manShort.set(RobotConstants.OpperarightStick);
  }

  left1.set(RobotConstants.DrivleftStick * RobotConstants.robotMaxSpeed);
  left2.set(RobotConstants.DrivrightStick * RobotConstants.robotMaxSpeed);
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
