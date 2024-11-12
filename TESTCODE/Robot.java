// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class driveTrain{
    // initialzing vars
    VictorSPX LeftMotor1;
    VictorSPX LeftMotor2;

    VictorSPX RightMotor1;
    VictorSPX RightMotor2;
    //ControlMode1 ControlMode;

    // constructor
    public driveTrain(VictorSPX LM1, VictorSPX LM2, VictorSPX RM1, VictorSPX RM2){
        LeftMotor1 = LM1;
        LeftMotor2 = LM2;

        RightMotor1 = RM1;
        RightMotor2 = RM2;
        //control = new ControlMode1();
    }
    // method
    void example(){
        // ooohhhhh stuff
    }
    void tankDrive(double LMS, double RMS){
        // change format for motor controler
        LeftMotor1.set(ControlMode.PercentOutput, LMS - 0.04 - (LMS/25));
        LeftMotor2.set(ControlMode.PercentOutput, LMS - 0.04 - (LMS/25));

        RightMotor1.set(ControlMode.PercentOutput, -RMS);
        RightMotor2.set(ControlMode.PercentOutput, -RMS);
    }
    void arcadeDrive(double Speed, double turnAng){
        // speed - sin turn ?
        // turn rad
        // neg turn ang left, pos right, 0 no turn
        /*
         * need width of robot(dist from centre to wheels)?
         * 
         * inner circ/outer
         */
        //turnAng *= -1; // just in case
        double Ls = 1;
        double Rs = 1;
        if (turnAng < 0){
            //turn left
            // left speed - sin(turn*pi/2)
            //Ls -= Math.sin(turnAng*Math.PI/2);
            Ls = Math.cos(turnAng*Math.PI);
        }
        else if(turnAng > 0){
            // turn Right
            // rihgt speed - sin(-turn*pi/2)
            //Rs -= Math.sin(-turnAng*Math.PI/2);
            Rs = Math.cos(turnAng*Math.PI);
        }
        else{
            // dont turn
        }
        tankDrive(Ls*Speed, Rs*Speed);
    }
    void pointShoot(double desiredAng, double currentAng, double speed){
        // curAng - desAng?
        arcadeDrive(speed, (((currentAng - desiredAng)/180)-1)/Math.PI);
        // will this work?
    }
}


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  VictorSPX motorL2 = new VictorSPX(7);
  VictorSPX motorR1 = new VictorSPX(5);
  VictorSPX motorR2 = new VictorSPX(6);
  VictorSPX motorL1 = new VictorSPX(10);

  Joystick flightStick = new Joystick(0);

  driveTrain drive = new driveTrain(motorL1, motorL2, motorR1, motorR2);
  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    double LSSpeed = flightStick.getRawAxis(0) * -1;
    double RSSpeed = flightStick.getRawAxis(1) * -1;
    SmartDashboard.putNumber("LeftSpeed", LSSpeed);
    SmartDashboard.putNumber("RightSpeed", RSSpeed);
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {
    /*
    motorL1.set(ControlMode.PercentOutput, .2);
    motorL2.set(ControlMode.PercentOutput, .2);

    motorR1.set(ControlMode.PercentOutput, -.2);
    motorR2.set(ControlMode.PercentOutput,-.2);
     */
    //drive.arcadeDrive(LSSpeed, 0);// trun angle in arcade dirve should vary from -1 to 1, automatically converts to angle
    //drive.pointShoot(90, 0, LSSpeed); // give angle in degrees (can be made for rad)
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    double LSSpeed = flightStick.getRawAxis(0) * -1;
    double RSSpeed = flightStick.getRawAxis(1) * -1;
    SmartDashboard.putNumber("LeftSpeed", LSSpeed);
    SmartDashboard.putNumber("RightSpeed", RSSpeed);
    System.out.println(LSSpeed);
    drive.tankDrive(LSSpeed, RSSpeed);
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