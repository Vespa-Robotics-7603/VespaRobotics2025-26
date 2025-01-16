// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

import frc.robot.components.Constants;
import frc.robot.components.DriveTrain;


/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    double speed;

    DriveTrain robot = new DriveTrain(
        Constants.moduleFL, // Front Left -> 10 & 11, CANcoder ID: 1
        Constants.moduleFR, // Front Right -> 40 & 41, CANcoder ID: 4
        Constants.moduleRL, // Rear Left -> 20 & 21, CANcoder ID: 2
        Constants.moduleRR  // Rear Right -> 30 & 31, CANcoder ID: 3
    );

    public static double[] getPolarSqr(double xPos, double yPos){
        /*
         * This will convert the coords from a square radius 1, to polar coords radius 1
         * Basicaly gives us the angle and dist from origin when given the coords from joystick (circle-ing the square)
         */
        // to have 0 degrees be forward x and y axis are swaped in atan 2
        double ang = Math.atan2(xPos,yPos);
        /*if (ang < 0 && false){
                ang += 2*Math.PI;
                // was to have vary from 0 to 2pi instead of -pi to pi, but I think - pi to pi is best
        }*/
        //System.out.println("angle");
        //System.out.println(ang);
        double hyp = Math.hypot(xPos, yPos);
        /*
         * to get relative radius reference expected rad for square
         * divide circle rad by sqr rad 
         * (I have a graph on desmos that makes regular polygons from angle, I'm using that)
         * sqr rad = 1/ cos (mod theta + pi/side of 2pi / side ~~a.k.a (theta + pi/side )% 2pi/side~~ - pi/side) ~~side is the number of sides, 4 in this case~~
         * circ rad = sqrt( x^2 + y^2 ) aka the hyp var
         * circ rad/sqr rad ( since sqr rad is 1 over cos, I can just multiply by cos)
         */
        double side = 4;// this var will never need to change
        //double modulusThing = (ang+Math.PI/side) - (2*Math.PI/side)*Math.floor( (ang+Math.PI/side) / (2*Math.PI/side) ); // mod function is x - yFloor(x/y), where x is first num and y is second ( this is the same as x mod y or x % y or mod x of y or mod(x,y) )
        double squareRad = Math.cos( ( (ang+Math.PI/side) % (2*Math.PI/side) - Math.PI/side) );
        /*
        System.out.println("Mod?");
        System.out.println( (ang+Math.PI/side) % (2*Math.PI/side));
        System.out.println("This should be squre rad");
        System.out.println(1/squareRad);
        */

        // ang is in rad, need it in deg
        ang = Math.toDegrees(ang);

        double[] polarCoords = {hyp*squareRad, ang};
        //System.out.println(polarCoords[0]);
        return polarCoords;
    }
    
    @Override
    public void robotInit() {}
    public Robot() {}
    
    @Override
    public void robotPeriodic() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {
        //speed = getPolarSqr(-Constants.JOYSTICKS.getRawAxis(0), Constants.VERTICAL_MOVEMENT)[0] * Math.signum(Constants.VERTICAL_MOVEMENT);
        speed = -Math.abs((Constants.JOYSTICKS.getRawAxis(0)) + (Constants.JOYSTICKS.getRawAxis(1)))*Math.signum(Constants.JOYSTICKS.getRawAxis(1));
        if(speed >= 0.9){
            speed = 1;
        } else if (speed <= -0.9){
            speed = -1;
        }
        robot.setSpeed(speed/5);
        robot.setDriveDirection(-Constants.JOYSTICKS.getRawAxis(0)*Math.signum(-Constants.JOYSTICKS.getRawAxis(1)));
        robot.setTurn(-Constants.JOYSTICKS.getRawAxis(4)/2);
        // triggers to control speed

        // switch (Constants.JOYSTICKS.getPOV()) {
        //     // gets dpad, counts 0 -> 315 degrees clockwise, returns -1 when not pressed
        //     case 0:
        //         robot.setDriveDirection(0); // up on dpad
        //         break;
        //     case 90:
        //         robot.setDriveDirection(0.25); // right on dpad
        //         break;
        //     case 180:
        //         robot.setDriveDirection(0.75); // down on dpad
        //         break;
        //     case 270:
        //         robot.setDriveDirection(1); // left on dpad
        //         break;
        //     case -1:
        //         // else: run left stick and right stick movement
        //         break;
        // }

        // double coder1reading = coder1.getPosition().getValueAsDouble();
        // double coder3reading = coder3.getPosition().getValueAsDouble();
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
