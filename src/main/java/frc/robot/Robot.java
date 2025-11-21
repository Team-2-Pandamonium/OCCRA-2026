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
  public final BooleanSupplier AtBottom = () -> CarrigeBottom.get();
  public final BooleanSupplier ElevatorHeightTooMuch = () -> elevatorEnc.getPosition() > 68;
  public final BooleanSupplier ELevatorNormalHeight = () -> (elevatorEnc.getPosition() <= 68 && elevatorEnc.getPosition() >= 4);
  public final BooleanSupplier ElevatorSLowDownHeight = () -> elevatorEnc.getPosition() <= 4;
  public final BooleanSupplier AtTop = (() -> (CarrigeTop.get() || (!stg2Top.get())));
  public final BooleanSupplier OverExtend = () -> (OPPERA_CONTROLLER.getPOV() == 0 || OPPERA_CONTROLLER.getPOV() == 45 || OPPERA_CONTROLLER.getPOV() == 315) && (AtTop.getAsBoolean());
  public final BooleanSupplier UnderExtend = () -> (OPPERA_CONTROLLER.getPOV() == 180 || OPPERA_CONTROLLER.getPOV() == 225 || OPPERA_CONTROLLER.getPOV() == 135) && CarrigeBottom.get();
  public final BooleanSupplier SlowMode = () -> DRIV_CONTROLLER.getL2Button();
  public final BooleanSupplier TurboMode = () -> DRIV_CONTROLLER.getL1Button();
  public final BooleanSupplier NormalMode = () -> (SlowMode.getAsBoolean() && TurboMode.getAsBoolean()) || (!SlowMode.getAsBoolean() && !TurboMode.getAsBoolean());
  public final BooleanSupplier Backwards = (() -> ((DRIV_CONTROLLER.getLeftY() > .01) && (DRIV_CONTROLLER.getLeftY() > .01)));
  public final BooleanSupplier NotBackwards = () -> DRIV_CONTROLLER.getLeftY() <= -.01 && DRIV_CONTROLLER.getLeftY() <= -.01;
  public final BooleanSupplier Turning = () -> (DRIV_CONTROLLER.getRightY() > .75 && DRIV_CONTROLLER.getLeftY() < -.75) || (DRIV_CONTROLLER.getRightY() < -.75 && DRIV_CONTROLLER.getLeftY() > .75);
  public final BooleanSupplier OperaUp = () -> ((OPPERA_CONTROLLER.getPOV() == 0) || (OPPERA_CONTROLLER.getPOV() == 315) || (OPPERA_CONTROLLER.getPOV() == 45));
  public final BooleanSupplier OperaDown = () -> ((OPPERA_CONTROLLER.getPOV() == 180) || (OPPERA_CONTROLLER.getPOV() == (180-45)) || (OPPERA_CONTROLLER.getPOV() == (180+45)));


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


  private SlewRateLimiter leftLimiter = new SlewRateLimiter(.5);  // 3 units/sec
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
    configR1.idleMode(IdleMode.kCoast).smartCurrentLimit(55).disableFollowerMode().inverted(false);
    SparkMaxConfig configR2 = new SparkMaxConfig();
    configR2.idleMode(IdleMode.kCoast).smartCurrentLimit(55).inverted(false).follow(right1);
    right1.configure(configR1, null, null);
    right2.configure(configR2, null, null);

    SparkMaxConfig configL1 = new SparkMaxConfig();
    configL1.idleMode(IdleMode.kCoast).smartCurrentLimit(55).disableFollowerMode().inverted(true);
    SparkMaxConfig configL2 = new SparkMaxConfig();
    configL2.idleMode(IdleMode.kCoast).smartCurrentLimit(55).inverted(true).follow(left1);
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
    UpdatePeriodic.updateSensorValues();
    updatePeriodic.updateControllerInputs();
    }
  

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    mode = "auton";
    gyro.reset();
    // starting and reseting the timer used in auton
    autonTimer.reset();
    // pid (right)
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (autonTimer.get() >= 1 && autonTimer.get() <= 3.5){
      left1.set(-.3);
      right1.set(-.3);
    } else {
      left1.set(0);
      right1.set(0);
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


    manipulator.setDefaultCommand(
      Commands.run(() -> {
          manLeft.set(Math.abs(OPPERA_CONTROLLER.getLeftY()) * OPPERA_CONTROLLER.getLeftY());
          manRight.set(Math.abs(OPPERA_CONTROLLER.getRightY()) * OPPERA_CONTROLLER.getRightY());
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
    // updatePeriodic.ABXYDpadUpdate();
    updatePeriodic.updateControllerInputs();
    // newTabKevin.add("Elevator Height ", RobotConstants.elevatorHeight);

    // ELEVATOR
    if (CarrigeBottom.get() == true) {
      elevatorEnc.setPosition(0);
    } 

    if (OperaUp.getAsBoolean()) {
      if (AtTop.getAsBoolean() == true){
        elevatorR.set(.03);
      } else if (ElevatorHeightTooMuch.getAsBoolean() == true){
        elevatorR.set(.1);
      } else if (ElevatorSLowDownHeight.getAsBoolean() == true){
        elevatorR.set(.2);
      } else {
        elevatorR.set(.55);
      }
    } else if (OperaDown.getAsBoolean() || OPPERA_CONTROLLER.getRightBumperButton()) {
      if (AtBottom.getAsBoolean() == true){
        elevatorR.set(-.03);
      } else if (ElevatorSLowDownHeight.getAsBoolean() == true){
        elevatorR.set(.1);
      } else {
        elevatorR.set(-.55);
      }
    }

    if (OPPERA_CONTROLLER.getLeftTriggerAxis() >.1 ) {
      manipulator.SetManipulators(.3 * OPPERA_CONTROLLER.getLeftTriggerAxis());
    } else if (OPPERA_CONTROLLER.getRightTriggerAxis() >.1) {
      manipulator.SetManipulators(-.3 * OPPERA_CONTROLLER.getRightTriggerAxis());
    }



      if (!(DRIV_CONTROLLER.getL2Axis() > .01 || DRIV_CONTROLLER.getR2Axis() > .01)){
        RobotConstants.GoFromLeftTrigger = false;
        RobotConstants.GoFromRightTrigger = false;
      }

      if (Turning.getAsBoolean()) { 
        RobotConstants.robotAccMaxSpeed = 1;

      } else if (DRIV_CONTROLLER.getL2Axis() > -.9){
        RobotConstants.GoFromLeftTrigger = true;

      } else if (DRIV_CONTROLLER.getR2Axis() > -.9){
        RobotConstants.GoFromRightTrigger = true;

      } else if (!(DRIV_CONTROLLER.getR1Button()) && !(DRIV_CONTROLLER.getL1Button())){
        RobotConstants.robotAccMaxSpeed = .6;
        System.err.println("Normaling It");


      } else if (DRIV_CONTROLLER.getR1Button()) {
        RobotConstants.robotAccMaxSpeed = 1;
        System.err.println("Turboing It");


      } else if (DRIV_CONTROLLER.getR1Button()) {
        RobotConstants.robotAccMaxSpeed = .125;
      }




      if (RobotConstants.GoFromLeftTrigger ){
      left1.set((-DRIV_CONTROLLER.getL2Axis()*RobotConstants.robotAccMaxSpeed+1)/2);
      right1.set((-DRIV_CONTROLLER.getL2Axis()*RobotConstants.robotAccMaxSpeed+1)/2);
    } else if (RobotConstants.GoFromRightTrigger ){
      left1.set((DRIV_CONTROLLER.getR2Axis()*RobotConstants.robotAccMaxSpeed+1)/2);
      right1.set((DRIV_CONTROLLER.getR2Axis()*RobotConstants.robotAccMaxSpeed+1)/2);
    } else {
      left1.set(Math.abs(DRIV_CONTROLLER.getLeftY())*DRIV_CONTROLLER.getLeftY()*RobotConstants.robotAccMaxSpeed);
      right1.set(Math.abs(DRIV_CONTROLLER.getRightY())*DRIV_CONTROLLER.getRightY()*RobotConstants.robotAccMaxSpeed);
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
