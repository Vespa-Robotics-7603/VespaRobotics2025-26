// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

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
  SwerveModule moduleFL = new SwerveModule(new TalonFX(10),new TalonFX(11),new CANcoder(1),0);
  SwerveModule moduleFR = new SwerveModule(new TalonFX(40),new TalonFX(41),new CANcoder(4),0);
  SwerveModule moduleBL = new SwerveModule(new TalonFX(20),new TalonFX(21),new CANcoder(2),0);
  SwerveModule moduleBR = new SwerveModule(new TalonFX(30),new TalonFX(31),new CANcoder(3),0);
  Drivetrain robot = new Drivetrain(moduleFL,moduleFR,moduleBL,moduleBR);
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
      //moduleFL.TurnTo(controller.getRawAxis(1));
      robot.setDriveDirection(joysticks.getRawAxis(2));
      robot.setSpeed(joysticks.getRawAxis(1)/2);
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
