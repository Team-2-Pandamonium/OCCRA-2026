package frc.robot.commands;

import java.util.Set;

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
    /**
     * Set's bool to TF (True or False)
     */
    public Command SetBool(boolean bool, boolean TF) {
        bool=TF;
        return this.run(()->{});
    }

    public Command SetDouble(double doubIN, double doubOUT) {
        doubIN=doubOUT;
        return this.run(()->{});
    }

    public Command RunCommand(Command command) {
        return this.run(() -> {command.schedule();});
    }

    /**
     * Gets the value from the controllers and updates it in the code
     */
    public void updateControllerInputs() {
        // if any of these are below the deadzone, it equals zero. abs so the controller
        // can go negative
        //Driver sticks
        Robot.DRIV_CONTROLLER.axisMagnitudeGreaterThan(1, RobotConstants.Drivdeadzone).whileFalse(SetDouble(RobotConstants.DrivleftStick, 0));
        Robot.DRIV_CONTROLLER.axisMagnitudeGreaterThan(1, RobotConstants.Drivdeadzone).whileTrue(SetDouble(RobotConstants.DrivleftStick, Robot.DRIV_CONTROLLER.getLeftY()));

        Robot.DRIV_CONTROLLER.axisMagnitudeGreaterThan(2, RobotConstants.Drivdeadzone).whileFalse(SetDouble(RobotConstants.DrivrightStick, 0));
        Robot.DRIV_CONTROLLER.axisMagnitudeGreaterThan(2, RobotConstants.Drivdeadzone).whileTrue(SetDouble(RobotConstants.DrivrightStick, Robot.DRIV_CONTROLLER.getRightY()));

        //Driver triggers
        Robot.DRIV_CONTROLLER.axisMagnitudeGreaterThan(3, RobotConstants.Drivdeadzone).whileFalse(SetDouble(RobotConstants.DrivleftTrigger, 0));
        Robot.DRIV_CONTROLLER.axisMagnitudeGreaterThan(3, RobotConstants.Drivdeadzone).whileTrue(SetDouble(RobotConstants.DrivleftTrigger, Robot.DRIV_CONTROLLER.getL2Axis()));

        Robot.DRIV_CONTROLLER.axisMagnitudeGreaterThan(4, RobotConstants.Drivdeadzone).whileFalse(SetDouble(RobotConstants.DrivrightTrigger, 0));
        Robot.DRIV_CONTROLLER.axisMagnitudeGreaterThan(4, RobotConstants.Drivdeadzone).whileTrue(SetDouble(RobotConstants.DrivrightTrigger, Robot.DRIV_CONTROLLER.getR2Axis()));

        //driver bumpers
        Robot.DRIV_CONTROLLER.L1().onTrue(SetBool(RobotConstants.DrivleftBumper, true));
        Robot.DRIV_CONTROLLER.L1().onFalse(SetBool(RobotConstants.DrivleftBumper, false));
        
        Robot.DRIV_CONTROLLER.R1().onTrue(SetBool(RobotConstants.DrivrightBumper, true));
        Robot.DRIV_CONTROLLER.R1().onFalse(SetBool(RobotConstants.DrivrightBumper, false));

        //operator triggers
        Robot.OPPERA_CONTROLLER.leftTrigger(RobotConstants.Oppdeadzone).whileFalse(SetDouble(RobotConstants.OpperaleftTrigger, 0));
        Robot.OPPERA_CONTROLLER.leftTrigger(RobotConstants.Oppdeadzone).whileTrue(SetDouble(RobotConstants.OpperaleftTrigger, RobotConstants.OpperaleftTrigger));

        Robot.OPPERA_CONTROLLER.rightTrigger(RobotConstants.Oppdeadzone).whileFalse(SetDouble(RobotConstants.OpperarightTrigger, 0));
        Robot.OPPERA_CONTROLLER.rightTrigger(RobotConstants.Oppdeadzone).whileTrue(SetDouble(RobotConstants.OpperarightTrigger, RobotConstants.OpperarightTrigger));
        
        //operator bumpers
        Robot.OPPERA_CONTROLLER.leftBumper().onTrue(SetBool(RobotConstants.OpperaleftBumper, true));
        Robot.OPPERA_CONTROLLER.leftBumper().onFalse(SetBool(RobotConstants.OpperaleftBumper, false));

        Robot.OPPERA_CONTROLLER.rightBumper().onTrue(SetBool(RobotConstants.OpperarightBumper, true));
        Robot.OPPERA_CONTROLLER.rightBumper().onFalse(SetBool(RobotConstants.OpperarightBumper, false));

        //buttons
        Robot.OPPERA_CONTROLLER.b().onTrue(SetBool(RobotConstants.OpperabButton, true));
        Robot.OPPERA_CONTROLLER.b().onFalse(SetBool(RobotConstants.OpperabButton, false));

        Robot.OPPERA_CONTROLLER.x().onTrue(SetBool(RobotConstants.OpperaxButton, true));
        Robot.OPPERA_CONTROLLER.x().onFalse(SetBool(RobotConstants.OpperaxButton, false));

        Robot.OPPERA_CONTROLLER.a().onTrue(SetBool(RobotConstants.OpperaaButton, true));
        Robot.OPPERA_CONTROLLER.a().onFalse(SetBool(RobotConstants.OpperaaButton, false));

        Robot.OPPERA_CONTROLLER.y().onTrue(SetBool(RobotConstants.OpperayButton, true));
        Robot.OPPERA_CONTROLLER.y().onFalse(SetBool(RobotConstants.OpperayButton, false));


        //dpad
        Robot.OPPERA_CONTROLLER.povUp().onTrue(SetBool(RobotConstants.OpperaDPadUp, true));
        Robot.OPPERA_CONTROLLER.povUp().onFalse(SetBool(RobotConstants.OpperaDPadUp, false));

        Robot.OPPERA_CONTROLLER.povUpRight().onTrue(SetBool(RobotConstants.OpperaDPadUpRight, true));
        Robot.OPPERA_CONTROLLER.povUpRight().onFalse(SetBool(RobotConstants.OpperaDPadUpRight, false));

        Robot.OPPERA_CONTROLLER.povRight().onTrue(SetBool(RobotConstants.OpperaDPadRight, true));
        Robot.OPPERA_CONTROLLER.povRight().onFalse(SetBool(RobotConstants.OpperaDPadRight, false));

        Robot.OPPERA_CONTROLLER.povDownRight().onTrue(SetBool(RobotConstants.OpperaDPadDownRight, true));
        Robot.OPPERA_CONTROLLER.povDownRight().onFalse(SetBool(RobotConstants.OpperaDPadDownRight, false));

        Robot.OPPERA_CONTROLLER.povLeft().onTrue(SetBool(RobotConstants.OpperaDPadLeft, true));
        Robot.OPPERA_CONTROLLER.povLeft().onFalse(SetBool(RobotConstants.OpperaDPadLeft, false));

        Robot.OPPERA_CONTROLLER.povUpLeft().onTrue(SetBool(RobotConstants.OpperaDPadUpLeft, true));
        Robot.OPPERA_CONTROLLER.povUpLeft().onFalse(SetBool(RobotConstants.OpperaDPadUpLeft, false));

    }

    public void ABXYDpadUpdate() {
        Robot.OPPERA_CONTROLLER.a().whileTrue(new Elevator().elevatorSetFancy(1));
        Robot.OPPERA_CONTROLLER.y().whileTrue(new Elevator().elevatorSetFancy(3));
        Robot.OPPERA_CONTROLLER.b().whileTrue(new Elevator().elevatorSetFancy(2));
    }

    public static void updateSensorValues() {
        RobotConstants.elevatorRotHeight = frc.robot.Robot.elevatorEnc.getPosition();
        RobotConstants.carrigeBot=frc.robot.Robot.CarrigeBottom.get();
        RobotConstants.carrigeTop=frc.robot.Robot.CarrigeTop.get();
        RobotConstants.stg2Top=frc.robot.Robot.stg2Top.get();
        RobotConstants.robotAngle=Robot.gyro.getAngle();
        // RobotConstants.elevatorHeight = Elevator.RottoIn(RobotConstants.elevatorRotHeight);
        if (RobotConstants.carrigeBot == false) {
            RobotConstants.bottEndstop = true;
            // System.out.println("at bottom");
        }else{
            RobotConstants.bottEndstop=false;
            // System.out.println("not at bottom");
        }

        if ((RobotConstants.stg2Top == false) && (RobotConstants.carrigeTop == false)) {
            RobotConstants.topEndstop = true;
            // System.out.println("at top");
        } else {
            RobotConstants.topEndstop = false;
            // System.out.println("not at top");
        }

    

    }




    public static void updateShuffleboardValues() {
        
    }
}
