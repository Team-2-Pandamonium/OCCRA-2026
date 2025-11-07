package frc.robot.commands;

public class Manipulator {

    /**
     * 
     * @param drvRot
     * @return <b>linVel<b> (inches)
     */
    public static double drvRotLinVel(double drvMotRot){
        double wheelRot=drvMotRot/8.46;
        return wheelRot*(Math.PI*6);
    }

    /**
     * @apiNote divide by circumference (diameter=4), and multiply by gear ratios (4)
     * @param LinVel
     * @return <b>manRot<b> (rot/sec)
     */
    public static double LinVeltoManRot(double LinVel){
        double wheelRot=LinVel/(Math.PI*4);
        return wheelRot*4;
    }
    
}