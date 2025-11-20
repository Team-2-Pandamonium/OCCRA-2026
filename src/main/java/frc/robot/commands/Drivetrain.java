package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;

public class Drivetrain extends SubsystemBase {
    public Command SetSlowMode() {
        return this.run(() -> {
            RobotConstants.slowMode = true;
        });
    };

    public Command SetTurboMode() {
        return this.run(() -> {
            RobotConstants.turboMode = true;
        });
    }

    public Command SetNormalMod() {
        return this.run(() -> {
            RobotConstants.turboMode = false;
            RobotConstants.slowMode = false;
        });
    }

    public Command GoFromLeftTrigger() {
        return this.run(() -> {
            RobotConstants.leftOutput = RobotConstants.DrivleftTrigger;
            RobotConstants.rightOutput = RobotConstants.DrivleftTrigger;
        });
    }

    public Command GoFromRightTrigger() {
        return this.run(() -> {
            RobotConstants.leftOutput = -RobotConstants.DrivleftTrigger;
            RobotConstants.rightOutput = -RobotConstants.DrivleftTrigger;
        });
    }
}
