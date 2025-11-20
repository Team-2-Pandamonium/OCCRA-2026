package frc.robot;

import frc.robot.commands.UpdatePeriodic;
import frc.robot.commands.AutoAlignment;
import frc.robot.commands.Auton;
import frc.robot.commands.Drivetrain;
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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import edu.wpi.first.wpilibj.SPI.Port;

import java.nio.ReadOnlyBufferException;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANrange;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class Robot extends TimedRobot {
  //motors
  public static final SparkMax manRight = new SparkMax(31, MotorType.kBrushless);
  public static final SparkMax manLeft = new SparkMax(32, MotorType.kBrushless);
  public static final SparkMax elevatorR = new SparkMax(21, MotorType.kBrushless);
  public static final SparkMax elevatorL = new SparkMax(22, MotorType.kBrushless);
  public static SparkMax right1 = new SparkMax(11, MotorType.kBrushless);
  public static SparkMax right2 = new SparkMax(12, MotorType.kBrushless);
  public static SparkMax left1 = new SparkMax(13, MotorType.kBrushless);
  public static SparkMax left2 = new SparkMax(14, MotorType.kBrushless);


  //PID encoders
  public static final RelativeEncoder drvLEnc = left1.getEncoder();
  public static final RelativeEncoder drvREnc = right1.getEncoder();

  //sensors
  public static final RelativeEncoder elevatorEnc = elevatorR.getEncoder();
  public static final CANrange backCanRange = new CANrange(0);
  public static final CANrange sideCanRange = new CANrange(1);

  //Feedforward
  public static SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0);




  //PID
  public static final SparkClosedLoopController elevatorRPID=manRight.getClosedLoopController(); //left follows


  public static final DigitalInput stg2Top = new DigitalInput(0);
  public static final DigitalInput CarrigeTop = new DigitalInput(1);
  public static final DigitalInput CarrigeBottom = new DigitalInput(2);

  //BooleanSuppliers  
  public final BooleanSupplier CarriageBottom2 = () -> CarrigeBottom.get();
  public final BooleanSupplier ElevatorHeightTooMuch = () -> RobotConstants.elevatorRotHeight > RobotConstants.maxHgtSlowThrthHld;
  public final BooleanSupplier NotElevatorHeightTooMuch = () -> RobotConstants.elevatorRotHeight <= RobotConstants.maxHgtSlowThrthHld;
  public final BooleanSupplier ElevatorSLowDownHeight = () -> RobotConstants.elevatorRotHeight <= RobotConstants.minDecelerationThreshold;
  public final BooleanSupplier OverExtend = () -> (RobotConstants.OpperaDPadUp || RobotConstants.OpperaDPadUpRight) && RobotConstants.topEndstop;
  public final BooleanSupplier UnderExtend = () -> (RobotConstants.OpperaDPadDown || RobotConstants.OpperaDPadDownRight) && RobotConstants.bottEndstop;
  public final BooleanSupplier NormalMode = () -> (RobotConstants.slowMode && RobotConstants.turboMode) || (!RobotConstants.slowMode && !RobotConstants.turboMode);
  public final BooleanSupplier SlowMode = () -> RobotConstants.slowMode;
  public final BooleanSupplier TurboMode = () -> RobotConstants.turboMode;
  public final BooleanSupplier AtTop = () -> RobotConstants.topEndstop || (RobotConstants.stg2Top == false);
  public final BooleanSupplier Backwards = () -> RobotConstants.DrivleftStick > 0 && RobotConstants.DrivrightStick > 0;
  public final BooleanSupplier NotBackwards = () -> RobotConstants.DrivleftStick <= 0 && RobotConstants.DrivrightStick <= 0;
  public final BooleanSupplier GoFasterButNotTurbo = () -> (RobotConstants.DrivrightStick > .75 && RobotConstants.DrivleftStick < -.75) || (RobotConstants.DrivrightStick < -.75 && RobotConstants.DrivleftStick > .75);


  // gyroscope
  public static final AHRS gyro = new AHRS(SPI.Port.kMXP);

  // camera
  public static final UsbCamera camera = CameraServer.startAutomaticCapture();
  // controllers
  public static final CommandPS5Controller DRIV_CONTROLLER = new CommandPS5Controller(0);
  public static final CommandPS5Controller OPPERA_CONTROLLER = new CommandPS5Controller(1);

  // Timers :(
  public static final Timer drivModeTimer=new Timer();
  public static final Timer autonTimer = new Timer();

  //Mode
  public String mode = "";

  //Shuffleboard
  // private ShuffleboardTab tab = Shuffleboard.getTab("Main");
  // public GenericEntry elevatorSetP = tab.addPersistent("ElevatorP",.001).getEntry();
  // public GenericEntry elevatorSetI = tab.addPersistent("ElevatorI",0).getEntry();
  // public GenericEntry elevatorSetD = tab.addPersistent("ElevatorD",.001).getEntry();
  // public SimpleWidget elevatorPosition = tab.add("Elevator Position", elevatorR.getEncoder());
  // public SimpleWidget roll = tab.add("Roll Angle", gyro.getRoll());
  // public SimpleWidget pitch = tab.add("Pitch Angle", gyro.getPitch());
  // public SimpleWidget yaw = tab.add("Yaw Angle", gyro.getYaw());

  //Command callers
  public AutoAlignment autoAlignment = new AutoAlignment();
  public Auton auton = new Auton();
  public Drivetrain drivetrain = new Drivetrain();
  public Elevator elevator = new Elevator();
  public Manipulator manipulator = new Manipulator();
  public UpdatePeriodic updatePeriodic = new UpdatePeriodic();


  private SlewRateLimiter leftLimiter = new SlewRateLimiter(3.0);  // 3 units/sec
  private SlewRateLimiter rightLimiter = new SlewRateLimiter(.5);


  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    //CANrange

    //SmartDashboard
    SmartDashboard.updateValues();
    SmartDashboard.setDefaultNumber("Elevator Encoder (rot)", elevatorEnc.getPosition());
    SmartDashboard.setDefaultNumber("Navx Roll", gyro.getRoll());
    SmartDashboard.setDefaultNumber("Navx Yaw", gyro.getYaw());
    SmartDashboard.setDefaultNumber("Navx Pitch", gyro.getPitch());
    SmartDashboard.setDefaultNumber("RPS of Left Motor", drvLEnc.getVelocity()*60);
    SmartDashboard.setDefaultNumber("RPS of Right Motor", drvREnc.getVelocity()*60);


    //SparkMaxConfig
    SparkMaxConfig configR1 = new SparkMaxConfig();
    configR1.idleMode(IdleMode.kCoast).smartCurrentLimit(40).disableFollowerMode().inverted(false);
    SparkMaxConfig configR2 = new SparkMaxConfig();
    configR2.idleMode(IdleMode.kCoast).smartCurrentLimit(40).inverted(false).follow(right1);
    right1.configure(configR1, null, null);
    right2.configure(configR2, null, null);

    SparkMaxConfig configL1 = new SparkMaxConfig();
    configL1.idleMode(IdleMode.kCoast).smartCurrentLimit(40).disableFollowerMode().inverted(true);
    SparkMaxConfig configL2 = new SparkMaxConfig();
    configL2.idleMode(IdleMode.kCoast).smartCurrentLimit(40).inverted(true).follow(left1);
    left1.configure(configL1, null, null);
    left2.configure(configL2, null, null);

    //ELEVATOR
    SparkMaxConfig configEleR = new SparkMaxConfig();
    configEleR.idleMode(IdleMode.kCoast).smartCurrentLimit(40).disableFollowerMode().inverted(false).closedLoop
    .pid(PIDVar.elevatorP,
        PIDVar.elevatorI,
        PIDVar.elevatorD,
        ClosedLoopSlot.kSlot0);
    SparkMaxConfig configEleL = new SparkMaxConfig();
    configEleL.idleMode(IdleMode.kCoast).smartCurrentLimit(40).inverted(false).follow(elevatorR, true).closedLoop
    .pid(PIDVar.elevatorP,
        PIDVar.elevatorI,
        PIDVar.elevatorD,
        ClosedLoopSlot.kSlot0);
    elevatorR.configure(configEleR, null, null);
    elevatorL.configure(configEleL, null, null);

    //MANIPULATOR
    SparkMaxConfig configManRight = new SparkMaxConfig();
    configManRight.idleMode(IdleMode.kCoast).smartCurrentLimit(40).disableFollowerMode().inverted(true);

    SparkMaxConfig configManLeft = new SparkMaxConfig();
    configManLeft.idleMode(IdleMode.kCoast).smartCurrentLimit(40).disableFollowerMode().inverted(false);
    manRight.configure(configManRight, null, null);
    manLeft.configure(configManLeft, null, null);

  }

  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    if (mode == "auton") {
      UpdatePeriodic.updateSensorValues();
    } else {
      UpdatePeriodic.updateSensorValues();
      new UpdatePeriodic().updateControllerInputs();
    }
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    mode = "auton";
    gyro.reset();
    // starting and reseting the timer used in auton
    autonTimer.reset();
    // pid (right)
    SparkMaxConfig configR1 = new SparkMaxConfig();
    configR1.idleMode(IdleMode.kCoast).smartCurrentLimit(40).disableFollowerMode().inverted(false);
    // .closedLoop
    // .pid(PIDVar.drvRightP,
    //     PIDVar.drvRightI,
    //     PIDVar.drvRightD,
    //     ClosedLoopSlot.kSlot0);
    SparkMaxConfig configR2 = new SparkMaxConfig();

    configR2.idleMode(IdleMode.kCoast).smartCurrentLimit(40).inverted(false).follow(right1);
    // .closedLoop
    // .pid(PIDVar.autonLeftP,
    //     PIDVar.autonLeftI,
    //     PIDVar.autonLeftD,
    //     ClosedLoopSlot.kSlot0);

    right1.configure(configR1, null, null);
    right2.configure(configR2, null, null);

    // pid (left)
    SparkMaxConfig configL1 = new SparkMaxConfig();
    configL1.idleMode(IdleMode.kCoast).smartCurrentLimit(40).disableFollowerMode().inverted(true);
    // .closedLoop
    // .pid(PIDVar.autonLeftP,
    //     PIDVar.autonLeftI,
    //     PIDVar.autonLeftD,
    //     ClosedLoopSlot.kSlot0);

    SparkMaxConfig configL2 = new SparkMaxConfig();
    configL2.idleMode(IdleMode.kCoast).smartCurrentLimit(40).inverted(true).follow(left1);
    // .closedLoop
    // .pid(PIDVar.autonLeftP,
    //     PIDVar.autonLeftI,
    //     PIDVar.autonLeftD,
    //     ClosedLoopSlot.kSlot0);

    left1.configure(configL1, null, null);
    left2.configure(configL2, null, null);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (autonConst.strt) {
      // move 1 ft (12 in)
      auton.intake().schedule();
      auton.goFwd(Auton.distToRot(156)).schedule();
      new WaitCommand(4).schedule();
      autonConst.movdStrt = true;
    }


    if (autonConst.movdStrt) {
      autonConst.strt = false;
      // turn to the shelf, 90 degrees
      auton.turnLR("L").schedule();
      new WaitCommand(3).schedule();
      autonConst.trnd = true;
      autonConst.movdStrt = false;
    }

    // go 12 in (1 ft) forwards
    if (autonConst.trnd) {
      auton.goFwd(Auton.distToRot(84)).schedule();
      new WaitCommand(4).schedule();
      autonConst.movToshelf = true;
    }

      // go to level 1 on elevator
      if (autonConst.movToshelf) {
        autonConst.trnd = false;
        auton.goToSpecificElevatorLevel(1).schedule();;
        new WaitCommand(2).schedule();;
      }

      // elevatorR.set(RobotConstants.elevatorOutput);

      // if the elevator is at level 1, then go forward 3 inches, into the shelf
      if (Math.abs(Elevator.CalcDist(1, RobotConstants.elevatorRotHeight) - elevatorEnc.getPosition()) <= 0.5) {
        autonConst.movToshelf = false;
        auton.goFwd(Auton.distToRot(3)).schedule();;
        new WaitCommand(1).schedule();;
        autonConst.push = true;
      }

      if (autonConst.push) {
        // release the CUBEEE
        autonConst.push = false;
        auton.outake().schedule();;
        new WaitCommand(1).schedule();;
        autonConst.outTaked = true;
      }

      if (autonConst.outTaked) {
        //move backwards 6 inches (0.5 ft)
        autonConst.outTaked = false;
        auton.goFwd(Auton.distToRot(-6)).schedule();;
        new WaitCommand(1).schedule();;
        autonConst.backedUp = true;
      }
      if (autonConst.backedUp) {
        // elevator bottom
        new Elevator().reset0(true).schedule();;
      }

      if (elevatorEnc.getPosition() == 0 && autonConst.backedUp) {
        // final condition, checking if elevator at bottom and backedUp is true, then prints finished
        autonConst.backedUp = false;
        for (int i = -1; i < 67; i++) {
          System.out.println("auton finished x"+Integer.toString(i));
        }
      }



  }

  /**
   * This function is called once each time the robot enters teleoperated mode.
   */
  @Override
  public void teleopInit() {
    mode = "teleop";
    // System.out.println("WARNING: RESETING ELEVATOR 0");
    // Elevator.reset0(false);
    // elevatorEnc.setPosition(0);
    drivModeTimer.start();
    drivModeTimer.reset();
    //unpid the pid (right)
    SparkMaxConfig configR1 = new SparkMaxConfig();
    configR1.idleMode(IdleMode.kBrake).smartCurrentLimit(50).disableFollowerMode().inverted(false);
    SparkMaxConfig configR2 = new SparkMaxConfig();
    configR2.idleMode(IdleMode.kBrake).smartCurrentLimit(50).inverted(false).follow(right1);
    right1.configure(configR1, null, null);
    right2.configure(configR2, null, null);

    //unpid the pid (left)
    SparkMaxConfig configL1 = new SparkMaxConfig();
    configL1.idleMode(IdleMode.kBrake).smartCurrentLimit(50).disableFollowerMode().inverted(true);
    SparkMaxConfig configL2 = new SparkMaxConfig();
    configL2.idleMode(IdleMode.kBrake).smartCurrentLimit(50).inverted(true).follow(left1);
    left1.configure(configL1, null, null);
    left2.configure(configL2, null, null);

    manipulator.setDefaultCommand(
      Commands.run(() -> {
          manLeft.set(Math.abs(RobotConstants.OpperaleftStick) * RobotConstants.OpperaleftStick);
          manRight.set(Math.abs(RobotConstants.OpperarightStick) * RobotConstants.OpperarightStick);
      }, manipulator)
  );


  elevator.setDefaultCommand(
    Commands.run(() -> {
        elevatorR.set(.03);
    }, elevator)
);


  }
  
  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    // UpdatePeriodic.updateControllerInputs();
    UpdatePeriodic.updateSensorValues();
    UpdatePeriodic.updateShuffleboardValues();
    updatePeriodic.ABXYDpadUpdate();
    updatePeriodic.updateControllerInputs();
    // newTabKevin.add("Elevator Height ", RobotConstants.elevatorHeight);

    // ELEVATOR
    if (RobotConstants.bottEndstop == true) {
      elevatorEnc.setPosition(0);
      // System.out.println("At bottom, reseting ENC zero");
    }    

    OPPERA_CONTROLLER.povUp().and(ElevatorHeightTooMuch).whileTrue(elevator.elevatorSetSpeed(0.1));
    
    OPPERA_CONTROLLER.povUp().and(NotElevatorHeightTooMuch).whileTrue(elevator.elevatorSetSpeed(.8));
    OPPERA_CONTROLLER.povUp().and(ElevatorSLowDownHeight).whileTrue(elevator.elevatorSetSpeed(.2));

    OPPERA_CONTROLLER.povUpRight().and(ElevatorHeightTooMuch).whileTrue(elevator.elevatorSetSpeed(0.05));
    OPPERA_CONTROLLER.povUpRight().and(NotElevatorHeightTooMuch).whileTrue(elevator.elevatorSetSpeed(0.2));

    OPPERA_CONTROLLER.povDown().whileTrue(elevator.elevatorSetSpeed(-0.5));

    OPPERA_CONTROLLER.povDownRight().whileTrue(elevator.elevatorSetSpeed(-0.1));

    Commands.run(() -> {elevatorR.set(0);}).onlyWhile(OverExtend);

    Commands.run(() -> {elevatorR.set(0);}).onlyWhile(UnderExtend);


  //Manipulator
    OPPERA_CONTROLLER.L2().whileTrue(manipulator.SetManipulators(RobotConstants.manMaxSPD * OPPERA_CONTROLLER.getL2Axis()));
    OPPERA_CONTROLLER.L2().whileTrue(manipulator.SetManipulators(-RobotConstants.manMaxSPD * OPPERA_CONTROLLER.getR2Axis()));

    OPPERA_CONTROLLER.circle().whileTrue(manipulator.RandomWeirdThingThatOperatorBButtonDoes());

    DRIV_CONTROLLER.L1().whileTrue(drivetrain.SetSlowMode());
    DRIV_CONTROLLER.R1().whileTrue(drivetrain.SetTurboMode());


    //Drivebase 


    Commands.run(() -> {
      RobotConstants.turboMode = false;
      RobotConstants.slowMode= false;
    }).onlyWhile(NormalMode);

    Commands.run(() -> {
      RobotConstants.turboMode = false;
      RobotConstants.slowModeMaxSpeed = 0.1;
    }).onlyWhile(Backwards);

    Commands.run(() -> {
      RobotConstants.slowModeMaxSpeed = 0.125;
    }).onlyWhile(NotBackwards);

    Commands.run(() -> {
      RobotConstants.robotAccMaxSpeed = RobotConstants.slowModeMaxSpeed;
    }).onlyWhile(SlowMode);

    Commands.run(() -> {
      RobotConstants.robotAccMaxSpeed = 1;
    }).onlyWhile(TurboMode);

    Commands.run(() -> {
      RobotConstants.robotAccMaxSpeed = RobotConstants.robotMaxSpeed;
    }).onlyWhile(NormalMode);

    Commands.run(() -> {
      RobotConstants.slowMode = true;
      RobotConstants.turboMode = false;
    }).onlyWhile(AtTop);

    DRIV_CONTROLLER.L2().whileTrue(drivetrain.GoFromLeftTrigger());
    DRIV_CONTROLLER.R2().whileTrue(drivetrain.GoFromRightTrigger());

    Commands.run(() -> {
      RobotConstants.rightOutput = RobotConstants.DrivrightStick * 20;
      RobotConstants.leftOutput = RobotConstants.DrivleftStick * 20;
      RobotConstants.robotAccMaxSpeed = 1;
    }).onlyWhile(GoFasterButNotTurbo);

  
    // DRIVE
    RobotConstants.leftOutput = Math.abs(RobotConstants.DrivleftStick) * RobotConstants.DrivleftStick;
    RobotConstants.rightOutput = Math.abs(RobotConstants.DrivrightStick) * RobotConstants.DrivrightStick;

    left1.set(leftLimiter.calculate(RobotConstants.leftOutput * RobotConstants.robotAccMaxSpeed));
    right1.set(rightLimiter.calculate(RobotConstants.rightOutput * RobotConstants.robotAccMaxSpeed));


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
