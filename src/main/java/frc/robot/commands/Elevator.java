package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Elevator extends SubsystemBase{

  public static final PIDController pidController = new PIDController(0, 0, 0);
  public static final SimpleMotorFeedforward ffController = new SimpleMotorFeedforward(1, 1);
  //Value holders, so the code doesn't cream in red lines
  private static final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);
  private static final MutAngle m_angle = Radians.mutable(0);
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);

  private final PIDController m_shooterFeedback = new PIDController(0.01, 0, 0);
  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(1, 1, 1);

  /**
   * This calculates how much the elevator has to move to get to the desired level
   * 
   * @apiNote 0 = ground
   * @apiNote 1 = 1st shelf
   * @apiNote 2 = 2nd shelf
   * @apiNote 3 = 3rd shelf
   * @param level
   * @param currHeight (in rotations)
   * @return <b>movDist<b> (in rotations)
   */
  public static double CalcDist(int level, double currHeight) {
    double desLevel = 0;
    switch (level) {
      case 0:
        desLevel = 0;
        break;
      case 1: // 1st shelf
        desLevel = RobotConstants.Level1;
        break;
      case 2: // 2nd shelf
        desLevel = RobotConstants.Level2;
        break;
      case 3: // 3rd shelf
        desLevel = RobotConstants.Level3;
        break;
      default: // else: return 0
        System.err.println("Error, invalid level number");
        break;
    }
    double movDist = InToRot(desLevel) - currHeight;
    return movDist;
  }

  /**
   * @param inches
   * @return <b>rotations<b>
   */
  public static double InToRot(double inches) {
    double cir = Math.PI * 2.256; // in inches
    double rot = inches / cir;
    rot *= 9; // gear ratio
    return rot;
  }

  /**
   * @param rot
   * @return <b>In<b>
   */
  public static double RotToIn(double rot) {
    rot /= 9; // gear ratio
    double cir = Math.PI * 2.256; // in inches
    double In = rot * cir;
    return In;
  }

  /**
  *
  * @param powered (use the motors to go to the 0 point or not)
  */
  public Command reset0(boolean powered) {
    BooleanSupplier CarriageBottom = () -> Robot.CarrigeBottom.get();
    return this.run(() -> {Robot.elevatorR.set(-0.3);})
    .onlyWhile(CarriageBottom)
    .finallyDo(() -> {Robot.elevatorR.set(0);
      Robot.elevatorEnc.setPosition(0);});
  // if (powered) {
  // while (Robot.CarrigeBottom.get()) {
  // Robot.elevatorR.set(-0.3);
  // }
  // Robot.elevatorR.set(0);
  // }
  // if (!RobotConstants.bottEndstop) {
  //   Robot.elevatorEnc.setPosition(0);
  // }  
  // }
  }
/**
 * Returns voltage from PID and FF calculations.
 * @param curVelocity current velocity of the motor
 * @param tarVelocity target velocity of the motor
 * @return
 */
  public static double getCalcVelocity(double curVelocity, double tarVelocity){
    return pidController.calculate(curVelocity, tarVelocity) + ffController.calculate(tarVelocity);
  }


  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              Robot.elevatorR::setVoltage,
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("ElevatorR")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            Robot.elevatorR.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(Robot.elevatorEnc.getPosition(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(Robot.elevatorEnc.getVelocity(), RotationsPerSecond));

                log.motor("ElevatorL")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            Robot.elevatorL.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(Robot.elevatorEnc.getPosition(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(-Robot.elevatorEnc.getVelocity(), RotationsPerSecond));
                
                log.motor("DrivetrainR")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            -Robot.right1.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(Robot.drvREnc.getPosition(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(Robot.drvREnc.getVelocity(), RotationsPerSecond));

                log.motor("DrivetrainL")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            Robot.left1.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(Robot.drvLEnc.getPosition(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(Robot.drvLEnc.getVelocity(), RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));
  
  
  /**
   * 
   * @param level the level for what level you want the elevator to be
   * @return elevator does shit
   */
  public Command elevatorSetFancy(int level) {
    BooleanSupplier condition = () -> CalcDist(level, RobotConstants.elevatorRotHeight)/RobotConstants.elevatorMaxHeight > 0.06;
    return run(() -> {Robot.elevatorR.set(CalcDist(level, RobotConstants.elevatorRotHeight)/RobotConstants.elevatorMaxRot);})
    .onlyWhile(condition);
  }

    

}