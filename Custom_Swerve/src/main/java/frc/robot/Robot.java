// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import java.time.OffsetDateTime;

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
  final double OFFSET = 0.350;
  CANcoder coder1 = new CANcoder(1);
  CANcoder coder3 = new CANcoder(3);
  SwerveModule moduleFL = new SwerveModule(new TalonFX(10),new TalonFX(11),new CANcoder(1),OFFSET,-1);
  SwerveModule moduleFR = new SwerveModule(new TalonFX(40),new TalonFX(41),new CANcoder(4),OFFSET,1);
  SwerveModule moduleBL = new SwerveModule(new TalonFX(20),new TalonFX(21),new CANcoder(2),OFFSET,-1);
  SwerveModule moduleBR = new SwerveModule(new TalonFX(30),new TalonFX(31),new CANcoder(3),OFFSET,1);
  DriveTrain robot = new DriveTrain(moduleFL,moduleFR,moduleBL,moduleBR);
  XboxController joysticks = new XboxController(0);
  @Override
  public void robotInit(){
    
  }
  public Robot() {
 
  }
  
  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
  }
  @Override
  public void teleopPeriodic() {
    switch (joysticks.getPOV()) {
        case 0:
            robot.setDriveDirection(OFFSET);
            break;
        case 45:
            robot.setDriveDirection(0.125 + OFFSET);
            break;
        case 90:
            robot.setDriveDirection(0.25 + OFFSET);
            break;
        case 135:
            robot.setDriveDirection(0.375 + OFFSET);
            break;
        case 180:
            robot.setDriveDirection(0.5 + OFFSET);
            break;
        case 225:
            robot.setDriveDirection(0.625 + OFFSET);
            break;
        case 270:
            robot.setDriveDirection(0.75 + OFFSET);
            break;
        case 315:
            robot.setDriveDirection(0.875 + OFFSET);
            break;
    
        default:
            break;
    }
    //moduleFL.TurnTo(controller.getRawAxis(1));
    robot.setDriveDirection(-joysticks.getRawAxis(0));
    robot.setSpeed((joysticks.getRawAxis(3)-joysticks.getRawAxis(2))/5);
    robot.setTurn(joysticks.getRawAxis(4));
    double coder1reading = coder1.getPosition().getValueAsDouble();
    double coder3reading = coder3.getPosition().getValueAsDouble();
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
