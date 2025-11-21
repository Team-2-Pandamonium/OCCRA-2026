package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;

public class Drivetrain extends SubsystemBase {
    public void SetSlowMode() {
        RobotConstants.slowMode = true;
    };

    public void SetTurboMode() {
            RobotConstants.turboMode = true;
    }

    public void SetNormalMod() {
            RobotConstants.turboMode = false;
            RobotConstants.slowMode = false;
    }

    public void GoFromLeftTrigger() {
        Robot.left1.set(Robot.DRIV_CONTROLLER.getL2Axis());
        Robot.right1.set(Robot.DRIV_CONTROLLER.getL2Axis());
    }

    public void GoFromRightTrigger() {
        Robot.left1.set(-Robot.DRIV_CONTROLLER.getL2Axis());
        Robot.right1.set(-Robot.DRIV_CONTROLLER.getL2Axis());
    }
}
