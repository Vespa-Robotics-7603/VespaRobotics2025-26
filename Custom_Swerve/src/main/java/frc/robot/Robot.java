// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.components.DriveTrain;
import frc.robot.components.SwerveModule;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;


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
    final static double OFFSET = 0.350;

    static CANcoder coder1 = new CANcoder(1);
    static CANcoder coder3 = new CANcoder(3);
    static XboxController joysticks = new XboxController(0);
    
    SwerveModule moduleFL = new SwerveModule(new TalonFX(10),new TalonFX(11),new CANcoder(1),OFFSET,-1);
    SwerveModule moduleFR = new SwerveModule(new TalonFX(40),new TalonFX(41),new CANcoder(4),OFFSET,1);
    SwerveModule moduleBL = new SwerveModule(new TalonFX(20),new TalonFX(21),new CANcoder(2),OFFSET,-1);
    SwerveModule moduleBR = new SwerveModule(new TalonFX(30),new TalonFX(31),new CANcoder(3),OFFSET,1);
    
    DriveTrain robot = new DriveTrain(moduleFL,moduleFR,moduleBL,moduleBR);
    
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
        robot.setSpeed((joysticks.getRawAxis(3)-joysticks.getRawAxis(2))/5);
        // triggers to control speed

        switch (joysticks.getPOV()) {
            // gets dpad, counts 0 -> 315 degrees clockwise, returns -1 when not pressed
            case 0:
                robot.setDriveDirection(0); // up on dpad
                break;
            case 90:
                robot.setDriveDirection(0.25); // right on dpad
                break;
            case 180:
                robot.setDriveDirection(0.75); // down on dpad
                break;
            case 270:
                robot.setDriveDirection(1); // left on dpad
                break;
            case -1:
                // else: run left stick and right stick movement
                robot.setDriveDirection(-joysticks.getRawAxis(0));
                robot.setTurn(joysticks.getRawAxis(4));
                break;
        }

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
