package frc.robot;

import frc.robot.commands.UpdatePeriodic;
import frc.robot.commands.Elevator;
// import frc.robot.constants.PIDVar;
import frc.robot.constants.RobotConstants;

import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.SparkClosedLoopController;

// import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;



public class Robot extends TimedRobot {

  //motors
  public static final SparkMax manShort = new SparkMax(31, MotorType.kBrushless);
  public static final SparkMax manLong = new SparkMax(32, MotorType.kBrushless);
  public static final SparkMax elevatorR = new SparkMax(21, MotorType.kBrushless);
  public static final SparkMax elevatorL = new SparkMax(22, MotorType.kBrushless);
  public static final SparkMax right1 = new SparkMax(11, MotorType.kBrushless);
  public static final SparkMax right2 = new SparkMax(12, MotorType.kBrushless);
  public static final SparkMax left1 = new SparkMax(13, MotorType.kBrushless);
  public static final SparkMax left2 = new SparkMax(14, MotorType.kBrushless);

  // REV PID loop
  // public static final SparkClosedLoopController elevatorRREV = elevatorR.getClosedLoopController();
  // public static final SparkClosedLoopController elevatorLREV = elevatorL.getClosedLoopController();
  //sensors
  public static final RelativeEncoder elevatorEnc = elevatorR.getEncoder();

  public static final DigitalInput stg2Top = new DigitalInput(0);
  public static final DigitalInput CarrigeTop = new DigitalInput(1);
  public static final DigitalInput CarrigeBottom = new DigitalInput(2);


  // controllers
  public static final XboxController DRIV_CONTROLLER = new XboxController(0);
  public static final XboxController OPPERA_CONTROLLER = new XboxController(1);

  // Timers :(
  public static final Timer drivModeTimer=new Timer();
  // shuffleboard
  // public ShuffleboardTab newTabKevin = Shuffleboard.getTab("KevinTabV2");
  // public GenericEntry cameraRequirement = newTabKevin.add("Camera
  // Requirements", 0).getEntry();
  // public GenericEntry elevatorheight = newTabKevin.add("Elevator Height: ",
  // RobotConstants.elevatorHeight).getEntry();

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
    configEleR.idleMode(IdleMode.kBrake).smartCurrentLimit(50).disableFollowerMode().inverted(false)/*
                                                                                                     * .closedLoop
                                                                                                     * .velocityFF(0)
                                                                                                     * .p(PIDVar.
                                                                                                     * elevatorRP,
                                                                                                     * ClosedLoopSlot.
                                                                                                     * kSlot0)
                                                                                                     * .i(PIDVar.
                                                                                                     * elevatorRI,
                                                                                                     * ClosedLoopSlot.
                                                                                                     * kSlot0)
                                                                                                     * .d(PIDVar.
                                                                                                     * elevatorRD,
                                                                                                     * ClosedLoopSlot.
                                                                                                     * kSlot0)
                                                                                                     */;
    SparkMaxConfig configEleL = new SparkMaxConfig();
    configEleL.idleMode(IdleMode.kBrake).smartCurrentLimit(50).inverted(false).follow(elevatorR, true)/*
                                                                                                       * .closedLoop
                                                                                                       * .velocityFF(0)
                                                                                                       * .p(PIDVar.
                                                                                                       * elevatorLP,
                                                                                                       * ClosedLoopSlot.
                                                                                                       * kSlot0)
                                                                                                       * .i(PIDVar.
                                                                                                       * elevatorLI,
                                                                                                       * ClosedLoopSlot.
                                                                                                       * kSlot0)
                                                                                                       * .d(PIDVar.
                                                                                                       * elevatorLD,
                                                                                                       * ClosedLoopSlot.
                                                                                                       * kSlot0)
                                                                                                       */;
    elevatorR.configure(configEleR, null, null);
    elevatorL.configure(configEleL, null, null);

    SparkMaxConfig configManShort = new SparkMaxConfig();
    configManShort.idleMode(IdleMode.kBrake).smartCurrentLimit(40).disableFollowerMode().inverted(true);
    SparkMaxConfig configManLong = new SparkMaxConfig();
    configManLong.idleMode(IdleMode.kBrake).smartCurrentLimit(40).disableFollowerMode().inverted(false);
    manShort.configure(configManShort, null, null);
    manLong.configure(configManLong, null, null);
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
    // Elevator.reset0(false);
    // elevatorEnc.setPosition(0);
    drivModeTimer.reset();
    drivModeTimer.start();
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
      System.out.println("At bottom, reseting ENC zero");
    }

      /*
       * if (RobotConstants.OpperaaButton &&
       * !(RobotConstants.OpperarightBumper)) { // lvl1
       * elevatorRREV.setReference(Elevator.CalcRot(1, RobotConstants.elevatorHeight),
       * ControlType.kPosition);
       * } else if (RobotConstants.OpperabButton &&
       * !RobotConstants.OpperarightBumper) { // lvl2
       * elevatorRREV.setReference(Elevator.CalcRot(2, RobotConstants.elevatorHeight),
       * ControlType.kPosition);
       * } else if (RobotConstants.OpperayButton &&
       * !RobotConstants.OpperarightBumper) { // lvl3
       * elevatorRREV.setReference(Elevator.CalcRot(3, RobotConstants.elevatorHeight),
       * ControlType.kPosition);
       * } else if (RobotConstants.OpperaxButton &&
       * !RobotConstants.OpperarightBumper) { // hp
       * elevatorRREV.setReference(Elevator.CalcRot(7, RobotConstants.elevatorHeight),
       * ControlType.kPosition);
       * } else if (RobotConstants.OpperaaButton &&
       * RobotConstants.OpperarightBumper) { // lvl1r
       * elevatorRREV.setReference(Elevator.CalcRot(4, RobotConstants.elevatorHeight),
       * ControlType.kPosition);
       * } else if (RobotConstants.OpperabButton
       * && RobotConstants.OpperarightBumper) { // lvl2 r
       * elevatorRREV.setReference(Elevator.CalcRot(5, RobotConstants.elevatorHeight),
       * ControlType.kPosition);
       * } else if (RobotConstants.OpperayButton
       * && RobotConstants.OpperarightBumper) { // lvl3 r
       * elevatorRREV.setReference(Elevator.CalcRot(6, RobotConstants.elevatorHeight),
       * ControlType.kPosition);
       * } else if (RobotConstants.OpperaleftBumper) {
       * elevatorRREV.setReference(Elevator.CalcRot(0, RobotConstants.elevatorHeight),
       * ControlType.kPosition);
       * }
       */
      // ABXY Elevator movment
      if (RobotConstants.OpperaaButton &&
          !(RobotConstants.OpperarightBumper)) { // lvl1
        RobotConstants.elevatorOutput = (Elevator.CalcRot(1, RobotConstants.elevatorRotHeight)
            / (RobotConstants.elevatorMaxRot));
        RobotConstants.otherInputMethod = true;

      } else if (RobotConstants.OpperabButton &&
          !RobotConstants.OpperarightBumper) { // lvl2
        RobotConstants.elevatorOutput = (Elevator.CalcRot(2, RobotConstants.elevatorRotHeight)
            / (RobotConstants.elevatorMaxRot));
        RobotConstants.otherInputMethod = true;

      } else if (RobotConstants.OpperayButton &&
          !RobotConstants.OpperarightBumper) { // lvl3
        RobotConstants.elevatorOutput = (Elevator.CalcRot(3, RobotConstants.elevatorRotHeight)
            / (RobotConstants.elevatorMaxRot));
        RobotConstants.otherInputMethod = true;

      } else if (RobotConstants.OpperaxButton &&
          !RobotConstants.OpperarightBumper) { // hp
        RobotConstants.elevatorOutput = (Elevator.CalcRot(7, RobotConstants.elevatorRotHeight)
            / (RobotConstants.elevatorMaxRot));
        RobotConstants.otherInputMethod = true;

      } else if (RobotConstants.OpperaaButton &&
          RobotConstants.OpperarightBumper) { // lvl1r
        RobotConstants.elevatorOutput = (Elevator.CalcRot(4, RobotConstants.elevatorRotHeight)
            / (RobotConstants.elevatorMaxRot));
        RobotConstants.otherInputMethod = true;

      } else if (RobotConstants.OpperabButton
          && RobotConstants.OpperarightBumper) { // lvl2 r
        RobotConstants.elevatorOutput = (Elevator.CalcRot(5, RobotConstants.elevatorRotHeight)
            / (RobotConstants.elevatorMaxRot));
        RobotConstants.otherInputMethod = true;

    } else if (RobotConstants.OpperayButton
        && RobotConstants.OpperarightBumper) { // lvl3 r
      RobotConstants.elevatorOutput = (Elevator.CalcRot(6, RobotConstants.elevatorRotHeight)
          / (RobotConstants.elevatorMaxRot));
      RobotConstants.otherInputMethod = true;

    } else if (RobotConstants.OpperaleftBumper) {
      RobotConstants.elevatorOutput = (Elevator.CalcRot(0, RobotConstants.elevatorRotHeight)
          / (RobotConstants.elevatorMaxRot));
      RobotConstants.otherInputMethod = true;

    } else {
      RobotConstants.otherInputMethod = false;
    }

    if (RobotConstants.otherInputMethod && Math.abs(RobotConstants.elevatorOutput) < RobotConstants.kPoffset) {
      RobotConstants.elevatorOutput *= 2;
    }

    // manual elevator movment
    if (RobotConstants.OpperaDPadUp) {
      if (RobotConstants.elevatorRotHeight > RobotConstants.maxHgtSlowThrthHld) {
        RobotConstants.elevatorOutput = (0.1);
      } else {
        RobotConstants.elevatorOutput = (0.3);
      }
    } else if (RobotConstants.OpperaDPadUpRight) {
      if (RobotConstants.elevatorRotHeight > RobotConstants.maxHgtSlowThrthHld) {
        RobotConstants.elevatorOutput = (0.05);

      } else {
        RobotConstants.elevatorOutput = (0.1);
      }
    } else if (RobotConstants.OpperaDPadDown) {
      RobotConstants.elevatorOutput = (-0.3);
    } else if (RobotConstants.OpperaDPadDownRight) {
      RobotConstants.elevatorOutput = (-0.1);
    } else if (RobotConstants.otherInputMethod == false) {
      RobotConstants.elevatorOutput = 0;
    }

    // not allowed to go over max speed
    if (Math.abs(RobotConstants.elevatorOutput) > RobotConstants.elevatorMaxSpeed) {
      if (RobotConstants.elevatorOutput < 0) {
        RobotConstants.elevatorOutput = -RobotConstants.elevatorMaxSpeed;
      } else if (RobotConstants.elevatorOutput > 0) {
        RobotConstants.elevatorOutput = RobotConstants.elevatorMaxSpeed;
      } else {
        System.out.println("ERROR: elevator max speed is 0");
      }
    }

  // digital stops
    if ((RobotConstants.OpperaDPadUp || RobotConstants.OpperaDPadUpRight) && RobotConstants.topEndstop == true) {
      elevatorR.set(0);
      System.err.println("ERROR: TRYING TO OVER EXTEND ELEVATOR, setting elevator speed to 0");
    } else if ((RobotConstants.OpperaDPadDown || RobotConstants.OpperaDPadDownRight)
        && RobotConstants.bottEndstop == true) {
      elevatorR.set(0);
      System.err.println("ERROR: TRYING TO UNDER EXTEND ELEVATOR, setting elevator speed to 0");
    } else {
      elevatorR.set(RobotConstants.elevatorOutput);
    }

  // MANIPULATOR
    if (RobotConstants.OpperarightTrigger > 0) { // intake
      manLong.set(RobotConstants.OpperarightTrigger * RobotConstants.manMaxSPD);
      manShort.set(RobotConstants.OpperarightTrigger * RobotConstants.manMaxSPD);
    } else if (RobotConstants.OpperaleftTrigger > 0) { // outtake
      manLong.set(-RobotConstants.OpperaleftTrigger * RobotConstants.manMaxSPD);
      manShort.set(-RobotConstants.OpperaleftTrigger * RobotConstants.manMaxSPD);
    } else { // manual control
      manLong.set(Math.abs(RobotConstants.OpperaleftStick) * RobotConstants.OpperaleftStick);
      manShort.set(Math.abs(RobotConstants.OpperarightStick) * RobotConstants.OpperarightStick);
    }

  // DRIVE
    if (drivModeTimer.get() >= 0.25) { // toggle drive mode
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
    } else {
      RobotConstants.robotAccMaxSpeed = RobotConstants.robotMaxSpeed;
    }

    if (RobotConstants.topEndstop) { // go to slow mode if elevator at top
      RobotConstants.turboMode = false;
      RobotConstants.slowMode = true;
    }

    if (RobotConstants.stg2Top == false) {
      RobotConstants.turboMode=false;
      RobotConstants.slowMode=true;
    }

    if (RobotConstants.turboMode==false) {
      left1
          .set((Math.abs(RobotConstants.DrivleftStick) * RobotConstants.DrivleftStick) * RobotConstants.robotAccMaxSpeed);
      right1.set(
          (Math.abs(RobotConstants.DrivrightStick) * RobotConstants.DrivrightStick) * RobotConstants.robotAccMaxSpeed);
    } else {
      left1.set(RobotConstants.DrivleftStick);
      right1.set(RobotConstants.DrivrightStick);
    }
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
