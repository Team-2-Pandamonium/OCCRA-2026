package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.constants.RobotConstants;

public class Elevator extends SubsystemBase {

    // Motors
    private final SparkMax elevatorR;
    private final SparkMax elevatorL;

    // Encoder
    private final RelativeEncoder encoder;

    // Endstops
    private final DigitalInput topEndstop;
    private final DigitalInput bottomEndstop;

    // Profiled PID controller
    private final ProfiledPIDController controller;

    // Manual output
    private double manualOutput = 0;

    public Elevator(SparkMax elevatorR, SparkMax elevatorL,
                    DigitalInput topEndstop, DigitalInput bottomEndstop) {
        this.elevatorR = elevatorR;
        this.elevatorL = elevatorL;
        this.topEndstop = topEndstop;
        this.bottomEndstop = bottomEndstop;

        // Left motor follows right
        elevatorL.follow(elevatorR, true);

        // Encoder from right motor
        encoder = elevatorR.getEncoder();

        // Profiled PID controller setup
        controller = new ProfiledPIDController(
                RobotConstants.elevatorKp, 0, 0,
                new TrapezoidProfile.Constraints(
                        RobotConstants.elevatorMaxVel,
                        RobotConstants.elevatorMaxAcc
                )
        );
        controller.setTolerance(0.01);
    }

    @Override
    public void periodic() {
        // Reset encoder at bottom endstop
        if (bottomEndstop.get()) {
            encoder.setPosition(0);
        }

        // Publish encoder value
        SmartDashboard.putNumber("Elevator Position", encoder.getPosition());

        // Run PID output if goal set
        if (!controller.atGoal()) {
            double pidOutput = controller.calculate(encoder.getPosition());
            elevatorR.set(pidOutput);
        } else {
            // Use manual input
            elevatorR.set(manualOutput);
        }
    }

    /** Manual control */
    public void setManual(double output) {
        manualOutput = output;
        // Clamp output for endstops
        if ((output > 0 && topEndstop.get()) || (output < 0 && bottomEndstop.get())) {
            manualOutput = 0;
        }
    }

    /** Move to a position */
    public void moveToPosition(double position) {
        controller.setGoal(position);
    }

    /** Check if at goal */
    public boolean atGoal() {
        return controller.atGoal();
    }

    /** Get current position */
    public double getPosition() {
        return encoder.getPosition();
    }

    /** Stop the elevator */
    public void stop() {
        elevatorR.set(0);
        manualOutput = 0;
    }
}