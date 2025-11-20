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
    public UpdatePeriodic updatePeriodic = new UpdatePeriodic();
    /**
     * Set's bool to TF (True or False)
     */
    public Command SetBool(BooleanConsumer setter, boolean TF) {
        return this.runOnce(()->{setter.accept(TF);});
    }

    public Command SetDouble(DoubleConsumer doubIN, double doubOUT) {
        return this.runOnce(()->{doubIN.accept(doubOUT);});
    }

    public Command RunCommand(Command command) {
        return this.runOnce(() -> {command.schedule();});
    }

    /**
     * Gets the value from the controllers and updates it in the code
     */
    public void updateControllerInputs() {
        // if any of these are below the deadzone, it equals zero. abs so the controller
        // can go negative
        //Driver sticks
        Robot.DRIV_CONTROLLER.axisMagnitudeGreaterThan(1, RobotConstants.Drivdeadzone).whileFalse(SetDouble(val -> RobotConstants.DrivleftStick = val, 0));
        Robot.DRIV_CONTROLLER.axisMagnitudeGreaterThan(1, RobotConstants.Drivdeadzone).whileTrue(SetDouble(val -> RobotConstants.DrivleftStick = val, Robot.DRIV_CONTROLLER.getLeftY()));

        Robot.DRIV_CONTROLLER.axisMagnitudeGreaterThan(2, RobotConstants.Drivdeadzone).whileFalse(SetDouble(val -> RobotConstants.DrivrightStick = val, 0));
        Robot.DRIV_CONTROLLER.axisMagnitudeGreaterThan(2, RobotConstants.Drivdeadzone).whileTrue(SetDouble(val -> RobotConstants.DrivrightStick = val, Robot.DRIV_CONTROLLER.getRightY()));

        //Driver triggers
        Robot.DRIV_CONTROLLER.axisMagnitudeGreaterThan(3, RobotConstants.Drivdeadzone).whileFalse(SetDouble(val -> RobotConstants.DrivleftTrigger = val, 0));
        Robot.DRIV_CONTROLLER.axisMagnitudeGreaterThan(3, RobotConstants.Drivdeadzone).whileTrue(SetDouble(val -> RobotConstants.DrivleftTrigger = val, Robot.DRIV_CONTROLLER.getL2Axis()));

        Robot.DRIV_CONTROLLER.axisMagnitudeGreaterThan(4, RobotConstants.Drivdeadzone).whileFalse(SetDouble(val -> RobotConstants.DrivrightTrigger = val, 0));
        Robot.DRIV_CONTROLLER.axisMagnitudeGreaterThan(4, RobotConstants.Drivdeadzone).whileTrue(SetDouble(val -> RobotConstants.DrivrightTrigger = val, Robot.DRIV_CONTROLLER.getR2Axis()));

        //driver bumpers
        Robot.DRIV_CONTROLLER.L1().onTrue(SetBool(val -> RobotConstants.DrivleftBumper = val, true));
        Robot.DRIV_CONTROLLER.L1().onFalse(SetBool(val -> RobotConstants.DrivleftBumper = val, false));
        
        Robot.DRIV_CONTROLLER.R1().onTrue(SetBool(val -> RobotConstants.DrivrightBumper = val, true));
        Robot.DRIV_CONTROLLER.R1().onFalse(SetBool(val -> RobotConstants.DrivrightBumper = val, false));

        //operator triggers
        Robot.OPPERA_CONTROLLER.leftTrigger(RobotConstants.Oppdeadzone).whileFalse(SetDouble(val -> RobotConstants.OpperaleftTrigger = val, 0));
        Robot.OPPERA_CONTROLLER.leftTrigger(RobotConstants.Oppdeadzone).whileTrue(SetDouble(val -> RobotConstants.OpperaleftTrigger = val, RobotConstants.OpperaleftTrigger));

        Robot.OPPERA_CONTROLLER.rightTrigger(RobotConstants.Oppdeadzone).whileFalse(SetDouble(val -> RobotConstants.OpperarightTrigger = val, 0));
        Robot.OPPERA_CONTROLLER.rightTrigger(RobotConstants.Oppdeadzone).whileTrue(SetDouble(val -> RobotConstants.OpperarightTrigger = val, RobotConstants.OpperarightTrigger));
        
        //operator bumpers
        Robot.OPPERA_CONTROLLER.leftBumper().onTrue(SetBool(val -> RobotConstants.OpperaleftBumper = val, true));
        Robot.OPPERA_CONTROLLER.leftBumper().onFalse(SetBool(val -> RobotConstants.OpperaleftBumper = val, false));

        Robot.OPPERA_CONTROLLER.rightBumper().onTrue(SetBool(val -> RobotConstants.OpperarightBumper = val, true));
        Robot.OPPERA_CONTROLLER.rightBumper().onFalse(SetBool(val -> RobotConstants.OpperarightBumper = val, false));

        //buttons
        Robot.OPPERA_CONTROLLER.b().onTrue(SetBool(val -> RobotConstants.OpperabButton = val, true));
        Robot.OPPERA_CONTROLLER.b().onFalse(SetBool(val -> RobotConstants.OpperabButton = val, false));

        Robot.OPPERA_CONTROLLER.x().onTrue(SetBool(val -> RobotConstants.OpperaxButton = val, true));
        Robot.OPPERA_CONTROLLER.x().onFalse(SetBool(val -> RobotConstants.OpperaxButton = val, false));

        Robot.OPPERA_CONTROLLER.a().onTrue(SetBool(val -> RobotConstants.OpperaaButton = val, true));
        Robot.OPPERA_CONTROLLER.a().onFalse(SetBool(val -> RobotConstants.OpperaaButton = val, false));

        Robot.OPPERA_CONTROLLER.y().onTrue(SetBool(val -> RobotConstants.OpperayButton = val, true));
        Robot.OPPERA_CONTROLLER.y().onFalse(SetBool(val -> RobotConstants.OpperayButton = val, false));


        //dpad
        Robot.OPPERA_CONTROLLER.povUp().onTrue(SetBool(val -> RobotConstants.OpperaDPadUp = val, true));
        Robot.OPPERA_CONTROLLER.povUp().onFalse(SetBool(val -> RobotConstants.OpperaDPadUp = val, false));

        Robot.OPPERA_CONTROLLER.povUpRight().onTrue(SetBool(val -> RobotConstants.OpperaDPadUpRight = val, true));
        Robot.OPPERA_CONTROLLER.povUpRight().onFalse(SetBool(val -> RobotConstants.OpperaDPadUpRight = val, false));

        Robot.OPPERA_CONTROLLER.povRight().onTrue(SetBool(val -> RobotConstants.OpperaDPadRight = val, true));
        Robot.OPPERA_CONTROLLER.povRight().onFalse(SetBool(val -> RobotConstants.OpperaDPadRight = val, false));

        Robot.OPPERA_CONTROLLER.povDownRight().onTrue(SetBool(val -> RobotConstants.OpperaDPadDownRight = val, true));
        Robot.OPPERA_CONTROLLER.povDownRight().onFalse(SetBool(val -> RobotConstants.OpperaDPadDownRight = val, false));

        Robot.OPPERA_CONTROLLER.povLeft().onTrue(SetBool(val -> RobotConstants.OpperaDPadLeft = val, true));
        Robot.OPPERA_CONTROLLER.povLeft().onFalse(SetBool(val -> RobotConstants.OpperaDPadLeft = val, false));

        Robot.OPPERA_CONTROLLER.povUpLeft().onTrue(SetBool(val -> RobotConstants.OpperaDPadUpLeft = val, true));
        Robot.OPPERA_CONTROLLER.povUpLeft().onFalse(SetBool(val -> RobotConstants.OpperaDPadUpLeft = val, false));

    }

    public void ABXYDpadUpdate() {
        Robot.OPPERA_CONTROLLER.a().whileTrue(elevator.elevatorSetFancy(1));
        Robot.OPPERA_CONTROLLER.y().whileTrue(elevator.elevatorSetFancy(3));
        Robot.OPPERA_CONTROLLER.b().whileTrue(elevator.elevatorSetFancy(2));
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
