package frc.robot.commands;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleConsumer;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;

public class UpdatePeriodic extends SubsystemBase {
    public AutoAlignment autoAlignment = new AutoAlignment();
    public Auton auton = new Auton();
    public Drivetrain drivetrain = new Drivetrain();
    public Elevator elevator = new Elevator();
    public Manipulator manipulator = new Manipulator();

    /**
     * Gets the value from the controllers and updates it in the code
     */
    public void updateControllerInputs() {
        //everything here was setting stuff for robotconstants
    }

    public void ABXYDpadUpdate() {
        while (Robot.OPPERA_CONTROLLER.getCrossButton()) {
            elevator.elevatorSetFancy(1);
        }
        while (Robot.OPPERA_CONTROLLER.getTriangleButton()) {
            elevator.elevatorSetFancy(3);
        }
        while (Robot.OPPERA_CONTROLLER.getCircleButton()) {
            elevator.elevatorSetFancy(2);
        }
    }

    public static void updateSensorValues() {
    }




    public static void updateShuffleboardValues() {
        
    }
}
