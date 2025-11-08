package frc.robot;

import frc.robot.commands.UpdatePeriodic;
import frc.robot.commands.Auton;
import frc.robot.commands.Elevator;
import frc.robot.commands.Manipulator;
import frc.robot.constants.PIDVar;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.autonConst;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI.Port;

import com.kauailabs.navx.frc.AHRS;

public class Robot extends TimedRobot {
  //motors
  public static final SparkMax manRight = new SparkMax(31, MotorType.kBrushless);
  public static final SparkMax manLeft = new SparkMax(32, MotorType.kBrushless);
  public static final SparkMax elevatorR = new SparkMax(21, MotorType.kBrushless);
  public static final SparkMax elevatorL = new SparkMax(22, MotorType.kBrushless);
  public static final SparkMax right1 = new SparkMax(11, MotorType.kBrushless);
  public static final SparkMax right2 = new SparkMax(12, MotorType.kBrushless);
  public static final SparkMax left1 = new SparkMax(13, MotorType.kBrushless);
  public static final SparkMax left2 = new SparkMax(14, MotorType.kBrushless);

  // PID loop
  public static final SparkClosedLoopController manLeftPID=manLeft.getClosedLoopController();
  public static final SparkClosedLoopController manRightPID=manRight.getClosedLoopController();

  //PID encoders
  public static final RelativeEncoder drvLEnc = left1.getEncoder();
  public static final RelativeEncoder drvREnc = right1.getEncoder();

  //sensors
  public static final RelativeEncoder elevatorEnc = elevatorR.getEncoder();

  public static final DigitalInput stg2Top = new DigitalInput(0);
  public static final DigitalInput CarrigeTop = new DigitalInput(1);
  public static final DigitalInput CarrigeBottom = new DigitalInput(2);

  // gyroscope
  public static final AHRS gyro = new AHRS(SPI.Port.kMXP);

  // camera
  public static final UsbCamera camera = CameraServer.startAutomaticCapture();

  // controllers
  public static final PS5Controller DRIV_CONTROLLER = new PS5Controller(0);
  public static final XboxController OPPERA_CONTROLLER = new XboxController(1);

  // Timers :(
  public static final Timer drivModeTimer=new Timer();
  public static final Timer autonTimer = new Timer();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
  
    //SparkMaxConfig
    SparkMaxConfig configR1 = new SparkMaxConfig();
    configR1.idleMode(IdleMode.kBrake).smartCurrentLimit(40).disableFollowerMode().inverted(false);
    SparkMaxConfig configR2 = new SparkMaxConfig();
    configR2.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(false).follow(right1);
    right1.configure(configR1, null, null);
    right2.configure(configR2, null, null);

    SparkMaxConfig configL1 = new SparkMaxConfig();
    configL1.idleMode(IdleMode.kBrake).smartCurrentLimit(40).disableFollowerMode().inverted(true);
    SparkMaxConfig configL2 = new SparkMaxConfig();
    configL2.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(true).follow(left1);
    left1.configure(configL1, null, null);
    left2.configure(configL2, null, null);

    SparkMaxConfig configEleR = new SparkMaxConfig();
    configEleR.idleMode(IdleMode.kBrake).smartCurrentLimit(40).disableFollowerMode().inverted(false);
    SparkMaxConfig configEleL = new SparkMaxConfig();
    configEleL.idleMode(IdleMode.kBrake).smartCurrentLimit(40).inverted(false).follow(elevatorR, true);
    elevatorR.configure(configEleR, null, null);
    elevatorL.configure(configEleL, null, null);

    SparkMaxConfig configManRight = new SparkMaxConfig();
    configManRight.idleMode(IdleMode.kBrake).smartCurrentLimit(40).disableFollowerMode().inverted(true).closedLoop
        .pid(PIDVar.manRightP,
            PIDVar.manRightI,
            PIDVar.manRightD,
            ClosedLoopSlot.kSlot0);
    SparkMaxConfig configManLeft = new SparkMaxConfig();
    configManLeft.idleMode(IdleMode.kBrake).smartCurrentLimit(40).disableFollowerMode().inverted(false).closedLoop
        .pid(PIDVar.manLeftP,
            PIDVar.manLeftI,
            PIDVar.manLeftD,
            ClosedLoopSlot.kSlot0);
    manRight.configure(configManRight, null, null);
    manLeft.configure(configManLeft, null, null);

  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    gyro.reset();
    // starting and reseting the timer used in auton
    autonTimer.reset();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    UpdatePeriodic.updateSensorValues();
    if (autonConst.strt) {
      autonConst.movdStrt = Auton.goFwd(Auton.distToRot(12));
    }


    if (autonConst.movdStrt) {
      autonConst.strt = false;
      if (gyro.getAngle() < 90) { // left auton
        right1.set(-RobotConstants.autonSpeed);
        left1.set(RobotConstants.autonSpeed);
      } else {
        right1.set(0);
        left1.set(0);
        autonConst.trnd = true;
        autonConst.movdStrt = false;
      }
    }

    // go x in forwards
    if (autonConst.trnd) {
      autonConst.movToshelf = Auton.goFwd(Auton.distToRot(12));
    }

      // go to x level on elevator
      if (autonConst.movToshelf) {
        autonConst.trnd = false;
        elevatorR.set(RobotConstants.elevatorOutput);
      RobotConstants.elevatorOutput = (Elevator.CalcDist(1, RobotConstants.elevatorRotHeight)
          / (RobotConstants.elevatorMaxRot));
      if (Math.abs(RobotConstants.elevatorOutput) > RobotConstants.elevatorMaxSpeed) {
        if (RobotConstants.elevatorOutput > 0) {
          RobotConstants.elevatorOutput = RobotConstants.elevatorMaxSpeed;
        } else {
          RobotConstants.elevatorOutput = -RobotConstants.elevatorMaxSpeed;
        }
      }

      }

      // elevatorR.set(RobotConstants.elevatorOutput);

      // if the elevator is at x level, then go forward 3 inches
      if (Math.abs(Elevator.CalcDist(1, RobotConstants.elevatorRotHeight) - elevatorEnc.getPosition()) <= 0.5) {
        autonConst.movToshelf = false;
        autonConst.push = Auton.goFwd(Auton.distToRot(3));
      }

      if (autonConst.push) {
        // release the CUBEEE
        manLeft.set(-RobotConstants.manMaxSPD);
        manRight.set(-RobotConstants.manMaxSPD);
        autonTimer.start();
      }

      if (autonTimer.get() > 0.5) {
        autonConst.outTaked = true;
        autonConst.push = false;
      }

      if (autonConst.outTaked) {
        autonConst.outTaked = false;
        autonConst.backedUp = Auton.goFwd(Auton.distToRot(-6));
      }
      if (autonConst.backedUp) {
        Elevator.reset0(true);
      }

      if (elevatorEnc.getPosition() == 0 && autonConst.backedUp) {
        autonConst.backedUp = false;
        for (int i = -1; i < 100; i++) {
          System.out.println("auton finished");
        }
      }



  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    // System.out.println("WARNING: RESETING ELEVATOR 0");
    // Elevator.reset0(false);
    // elevatorEnc.setPosition(0);
    drivModeTimer.start();
    drivModeTimer.reset();
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    UpdatePeriodic.updateControllerInputs();
    UpdatePeriodic.updateSensorValues();
    // newTabKevin.add("Elevator Height ", RobotConstants.elevatorHeight);

    // ELEVATOR
    if (RobotConstants.bottEndstop == true) {
      elevatorEnc.setPosition(0);
      // System.out.println("At bottom, reseting ENC zero");
    }

    // PID example
    // elevatorR.set(elevRpid.calculate(RobotConstants.elevatorRotHeight,
    // Elevator.InToRot(RobotConstants.Level1)));

    // AXY Elevator movment
    if (RobotConstants.OpperaaButton) { // lvl1
      RobotConstants.elevatorOutput = (Elevator.CalcDist(1, RobotConstants.elevatorRotHeight)
          / (RobotConstants.elevatorMaxRot));

    } else if (RobotConstants.OpperayButton) { // lvl3
      RobotConstants.elevatorOutput = (Elevator.CalcDist(3, RobotConstants.elevatorRotHeight)
          / (RobotConstants.elevatorMaxRot));

    } else if (RobotConstants.OpperaxButton) { // lvl2
      RobotConstants.elevatorOutput = (Elevator.CalcDist(2, RobotConstants.elevatorRotHeight)
          / (RobotConstants.elevatorMaxRot));

    } else if (RobotConstants.OpperaDPadUp) { // DPAD movment (manual)

      if (RobotConstants.elevatorRotHeight > RobotConstants.maxHgtSlowThrthHld) {
        RobotConstants.elevatorOutput = (0.1);
      } else {
        RobotConstants.elevatorOutput = (0.3);
      }
    } else if (RobotConstants.OpperaDPadUpRight) {

      if (RobotConstants.elevatorRotHeight > RobotConstants.maxHgtSlowThrthHld) {
        RobotConstants.elevatorOutput = 0.05;

      } else {
        RobotConstants.elevatorOutput = 0.1;
      }
    } else if (RobotConstants.OpperaDPadDown) {
      RobotConstants.elevatorOutput = -0.3;

    } else if (RobotConstants.OpperaDPadDownRight) {
      RobotConstants.elevatorOutput = -0.1;

    } else {
      RobotConstants.elevatorOutput = 0.03;
    }


    // not allowed to go over max speed
    if (Math.abs(RobotConstants.elevatorOutput) > RobotConstants.elevatorMaxSpeed) {
      if (RobotConstants.elevatorOutput < 0) {
        RobotConstants.elevatorOutput = -RobotConstants.elevatorMaxSpeed;
      } else if (RobotConstants.elevatorOutput > 0) {
        RobotConstants.elevatorOutput = RobotConstants.elevatorMaxSpeed;
      } else {
        System.err.println("ERROR: elevator max speed is 0");
      }
    }

    // digital stops
    if ((RobotConstants.OpperaDPadUp || RobotConstants.OpperaDPadUpRight) && RobotConstants.topEndstop == true) {
      RobotConstants.elevatorOutput = 0;
      System.err.println("ERROR: TRYING TO OVER EXTEND ELEVATOR, setting elevator speed to 0");

    } else if ((RobotConstants.OpperaDPadDown || RobotConstants.OpperaDPadDownRight)
        && RobotConstants.bottEndstop == true) {
      RobotConstants.elevatorOutput = 0;
      System.err.println("ERROR: TRYING TO UNDER EXTEND ELEVATOR, setting elevator speed to 0");
    }

    elevatorR.set(RobotConstants.elevatorOutput); // one of the only times the elevator speed actually gets set in the
                                                  // code

    // MANIPULATOR
    
    /* 
    RobotConstants.manLeftOutput=Math.abs(Manipulator.LinVeltoManRot(Manipulator.drvRotLinVel(drvLEnc.getVelocity()))); //always positive
    RobotConstants.manRightOutput=Math.abs(Manipulator.LinVeltoManRot(Manipulator.drvRotLinVel(drvREnc.getVelocity()))); //always positive
    if(RobotConstants.manLeftOutput==0){
      RobotConstants.manLeftOutput=1;
    }
    if(RobotConstants.manRightOutput==0){
      RobotConstants.manRightOutput=1;
    }

    if(RobotConstants.manLeftOutput<=0.1 && RobotConstants.manLeftOutput>=0){
      RobotConstants.manLeftOutput=0.10;
    } else if (RobotConstants.manLeftOutput>=-0.1 && RobotConstants.manLeftOutput<=0) {
      RobotConstants.manLeftOutput=-0.10;
    }
    if(RobotConstants.manRightOutput<=0.1 && RobotConstants.manRightOutput>=0){
      RobotConstants.manRightOutput=0.10;
    } else if (RobotConstants.manLeftOutput>=-0.1 && RobotConstants.manLeftOutput<=0) {
      RobotConstants.manRightOutput=-0.10;
    }
    
    */
    if (RobotConstants.OpperarightTrigger > 0) { // intake
      // manLeftPID.setReference(-RobotConstants.manLeftOutput*RobotConstants.manMaxSPD,ControlType.kVelocity);
      // manRightPID.setReference(-RobotConstants.manRightOutput*RobotConstants.manMaxSPD,ControlType.kVelocity);
      manLeft.set(RobotConstants.OpperarightTrigger*RobotConstants.manMaxSPD);
      manRight.set(RobotConstants.OpperarightTrigger*RobotConstants.manMaxSPD);

    } else if (RobotConstants.OpperaleftTrigger > 0) { // outtake
      // manLeftPID.setReference(RobotConstants.manLeftOutput*RobotConstants.manMaxSPD,ControlType.kVelocity);
      // manRightPID.setReference(RobotConstants.manRightOutput*RobotConstants.manMaxSPD,ControlType.kVelocity);
      manLeft.set(-RobotConstants.OpperaleftTrigger*RobotConstants.manMaxSPD);
      manRight.set(-RobotConstants.OpperaleftTrigger*RobotConstants.manMaxSPD);

    } else if (RobotConstants.OpperabButton) {
      // manLeftPID.setReference(-(RobotConstants.manLeftOutput/2)*RobotConstants.manMaxSPD,ControlType.kVelocity);
      // manRightPID.setReference(-(RobotConstants.manRightOutput/2)*RobotConstants.manMaxSPD,ControlType.kVelocity);
      manLeft.set(RobotConstants.manMaxSPD/2);
      manRight.set(-RobotConstants.manMaxSPD/2);

    } else { // manual control
      manLeft.set(Math.abs(RobotConstants.OpperaleftStick) * RobotConstants.OpperaleftStick);
      manRight.set(Math.abs(RobotConstants.OpperarightStick) * RobotConstants.OpperarightStick);

    }

    // DRIVE
    if (drivModeTimer.get() >= 0.1) { // toggle drive mode
    RobotConstants.slowMode ^= RobotConstants.DrivleftBumper;
    RobotConstants.turboMode ^= RobotConstants.DrivrightBumper;
    drivModeTimer.reset();
    }

    if (RobotConstants.slowMode && RobotConstants.turboMode) { // if both pressed, make it neither
      RobotConstants.turboMode = false;
      RobotConstants.slowMode = false;
    }

    if (RobotConstants.slowMode) { // slowmode max speed
      RobotConstants.robotAccMaxSpeed = RobotConstants.slowModeMaxSpeed;
    } else if (RobotConstants.turboMode) {
      RobotConstants.robotAccMaxSpeed = 1;
    } else {
      RobotConstants.robotAccMaxSpeed = RobotConstants.robotMaxSpeed;
    }

    if (RobotConstants.topEndstop || (RobotConstants.stg2Top == false)) { // slow mode if elevator or stage 2 at top
      RobotConstants.turboMode = false;
      RobotConstants.slowMode = true;
    }

    if (RobotConstants.DrivleftStick > 0 && RobotConstants.DrivrightStick > 0) { // if backwards make slow mode slower
      RobotConstants.turboMode = false;
      RobotConstants.slowModeMaxSpeed = 0.1;
    } else {
      RobotConstants.slowModeMaxSpeed = 0.125;
    }

    if (RobotConstants.DrivleftTrigger > 0) {
      RobotConstants.leftOutput = RobotConstants.DrivleftTrigger;
      RobotConstants.rightOutput = RobotConstants.DrivleftTrigger;
      System.out.println("going from left trigger");

    } else if (RobotConstants.DrivrightTrigger > 0) {
      RobotConstants.leftOutput = -RobotConstants.DrivrightTrigger;
      RobotConstants.rightOutput = -RobotConstants.DrivrightTrigger;
      System.out.println("going from right trigger");

    } else {
      RobotConstants.leftOutput = Math.abs(RobotConstants.DrivleftStick) * RobotConstants.DrivleftStick;
      RobotConstants.rightOutput = Math.abs(RobotConstants.DrivrightStick) * RobotConstants.DrivrightStick;
      System.out.println("going from normal controls");
    }

    left1.set(RobotConstants.leftOutput * RobotConstants.robotAccMaxSpeed);
    right1.set(RobotConstants.rightOutput * RobotConstants.robotAccMaxSpeed);
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
