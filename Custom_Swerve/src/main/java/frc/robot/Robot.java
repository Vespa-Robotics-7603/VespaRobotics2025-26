// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.components.Constants;
import edu.wpi.first.wpilibj.XboxController;



/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

    // CANcoders / "potentiometers"
    public static CANcoder coder1 = new CANcoder(1);
    public static CANcoder coder2 = new CANcoder(2);
    public static CANcoder coder3 = new CANcoder(3);
    public static CANcoder coder4 = new CANcoder(4);

    XboxController joysticks = new XboxController(0);    

    double speed;

    /**
     * This will convert the coords from a square radius 1, to polar coords radius 1.
     * Basicaly gives us the angle and dist from origin when given the coords from joystick (circle-ing the square)
     *
     * @param xPos x coord
     * @param yPos y coord
     * @return coords in {radius, angle in degrees}
      */
    public static double[] getPolarSqr(double xPos, double yPos){
        
        double ang = Math.atan2(xPos,yPos);
        
        double hyp = Math.hypot(xPos, yPos);
        
        double side = 4;
        double squareRad = Math.cos( ( (ang+Math.PI/side) % (2*Math.PI/side) - Math.PI/side) );
        // ang is in rad, need it in deg
        ang = Math.toDegrees(ang);
        double[] polarCoords = {hyp/*squareRad*/, ang};
        return polarCoords;
    }
    
    /**
     * This function is run when the Constants.ROBOT is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {}
    public Robot() {}
    
    @Override
    public void robotPeriodic() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {
        // lets test this please
        Constants.ROBOT.turnWheelsTo(90);
    }

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {
        // if (Constants.RIGHT_TRIGGER<Constants.TRIGGER_DEADZONE)

        // speed = getPolarSqr(-Constants.JOYSTICKS.getRawAxis(0), Constants.JOYSTICKS.getRawAxis(1))[0] * Math.signum(Constants.JOYSTICKS.getRawAxis(1));
        this.speed = -Math.abs((Constants.JOYSTICKS.getRawAxis(0)) + (Constants.JOYSTICKS.getRawAxis(1)))*Math.signum(Constants.JOYSTICKS.getRawAxis(1));

        if(speed >= 0.9){
            speed = 1;
        } else if (speed <= -0.9){
            speed = -1;
        }
        Constants.ROBOT.setSpeed(speed/5);
        Constants.ROBOT.setDriveDirection(-Constants.JOYSTICKS.getRawAxis(0)*Math.signum(-Constants.JOYSTICKS.getRawAxis(1)));
        Constants.ROBOT.setTurn(-Constants.JOYSTICKS.getRawAxis(4)/2);
    }

    @Override

    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
