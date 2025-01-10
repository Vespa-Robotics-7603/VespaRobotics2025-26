// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
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
  CANcoder coder = new CANcoder(1);
  TalonFX drivemotorFL = new TalonFX(10);
  TalonFX turnMotorFL = new TalonFX(11);
  SwerveModule moduleFL = new SwerveModule(drivemotorFL,turnMotorFL,coder);
  TalonFX drivemotorFR = new TalonFX(40);
  TalonFX drivemotorBL = new TalonFX(20);
  TalonFX drivemotorBR = new TalonFX(30);
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
    //turnMotorFL.set(0.4);
    System.out.println(moduleFL.getOffset());
    moduleFL.setDriveSpeed(0.1);
    moduleFL.TurnTo(90);    
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
