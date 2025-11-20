package frc.robot.commands;

import frc.robot.commands.UpdatePeriodic;
import frc.robot.Robot;
import frc.robot.commands.Auton;
import frc.robot.commands.Elevator;
import frc.robot.commands.Manipulator;
import frc.robot.constants.PIDVar;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.autonConst;

import java.util.function.BooleanSupplier;

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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.SPI.Port;

public class AutoAlignment extends SubsystemBase {
    /**
     * 
     * @param Angle
     * @return Turn's that angle (90 left, 270 right, 180 back, 0 front)
     */
    public Command TurnAngle(int Angle) {
        BooleanSupplier Condition = () -> Robot.gyro.getAngle() > Angle+0.05 || Robot.gyro.getAngle() < Angle-0.05;

        return this.run(() -> {
        Robot.right1.set(-Math.copySign(0.5, Angle - Robot.gyro.getAngle()));
        Robot.left1.set(Math.copySign(0.5, Angle - Robot.gyro.getAngle()));})
        .onlyWhile(Condition)
        .finallyDo(() -> {
        Robot.right1.set(0);
        Robot.left1.set(0);});
    }

    /**
     * 
     * @param DistInInches
     * @return Drives <b>FORWARD<b> that amount of inches
     */
    public Command DriveUntil(double DistInInches) {
        BooleanSupplier Condition = () -> Robot.backCanRange.getDistance().getValueAsDouble()*39.37 > DistInInches;

        return this.run(() -> {
        Robot.right1.set(-0.67);
        Robot.left1.set(-0.67);})
        .onlyWhile(Condition)
        .finallyDo(() -> {
        Robot.right1.set(0);
        Robot.left1.set(0);});
    }
    /**
     * run with 
     * @apiNote <b>IMPORTANT<b>; 
     * @apiNote "pos" values: 
     * @apiNote 1: rightmost spot on shelf
     * @apiNote 2: second rightmost spot on shelf
     * @apiNote 3: middle spot on shelf
     * @apiNote 4: second leftmost spot on shelf
     * @apiNote 5: leftmost spot on shelf
     * @param Pos
     * @return Auto Alignment For Shelf
     */
    public Command AutoAlignmentForShelf(int Pos) {
        double distanceToDrive = switch(Pos) {
            case 1 -> 156;
            case 2 -> 144;
            case 3 -> 132;
            case 4 -> 120;
            case 5 -> 108;
            default -> throw new IllegalArgumentException("pos must be 1-5");
        };
    
        return Commands.sequence(
            TurnAngle(180),
            DriveUntil(distanceToDrive),
            TurnAngle(90),
            DriveUntil(10)
        );
    }
}
