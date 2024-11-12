package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import java.util.ResourceBundle.Control;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Timer;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {

  private VictorSPX driveLeftSpark = new VictorSPX(1);
  private VictorSPX driveRightSpark = new VictorSPX(3);
  private VictorSPX driveLeftVictor = new VictorSPX(2);
  private VictorSPX driveRightVictor = new VictorSPX(4);
  private Joystick newJoystick = new Joystick(0);
  CANSparkMax armMotor = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax shooterMotorBase = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax shooterMotor1 = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax shooterMotor2 = new CANSparkMax(4, MotorType.kBrushless);
  // Add a variable to track whether the turn button was pressed in the previous iteration
  private boolean previousButtonState = false;

  // Timer to control the duration of the turn
  private double turnStartTime = 0.0;
  private double turnDuration = 2.0; // Adjust the duration as needed

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
  }

  @Override
  public void teleopPeriodic() {
    double x_ampl = newJoystick.getRawAxis(1);
    double y_ampl = newJoystick.getRawAxis(0);
    double leftSpeed = (-x_ampl + y_ampl);
    double rightSpeed = (y_ampl + x_ampl);
  
    if (newJoystick.getRawButton(1)){
      shooterMotor1.set(-5);
      shooterMotor2.set(-5);
    }
    else if (newJoystick.getRawButton(2)){
      shooterMotor1.set(5);
      shooterMotor2.set(5);
    }  
    else if(newJoystick.getRawButton(6)){
      shooterMotor2.set(-5);
    }
    else{
      shooterMotor1.set(0);
      shooterMotor2.set(0);
    }
    if (newJoystick.getRawButton(5)){
      shooterMotorBase.set(1);
    }
    else if(newJoystick.getRawButton(3)){
      shooterMotorBase.set(-1);
    }
    else{
      shooterMotorBase.set(0);
    }

    if (leftSpeed > 1) {
      leftSpeed = leftSpeed / 3;
    }

    if (rightSpeed > 1) {
      rightSpeed = rightSpeed / 3;   
    }

    // Check if the button is pressed and was not pressed in the previous iteration
    boolean currentButtonState = newJoystick.getRawButton(11);

    if (currentButtonState && !previousButtonState) {
      // Button was just pressed, initiate a 90-degree turn using a timer
      turnStartTime = Timer.getFPGATimestamp();
    }

    // Check if the turn duration has passed
    if (Timer.getFPGATimestamp() - turnStartTime < turnDuration) {
      // Continue turning
      driveLeftSpark.set(ControlMode.PercentOutput, 0.5);  // Example speed, adjust as needed
      driveRightSpark.set(ControlMode.PercentOutput, -0.5); // Example speed, adjust as needed
    } else {
      // Stop the motors once the turn is complete
      driveLeftSpark.set(ControlMode.PercentOutput, 0);
      driveRightSpark.set(ControlMode.PercentOutput, 0);
    }

    // Update the previous button state for the next iteration
    previousButtonState = currentButtonState;

    driveLeftSpark.set(ControlMode.PercentOutput, leftSpeed);
    driveLeftVictor.set(ControlMode.PercentOutput, leftSpeed);
    driveRightVictor.set(ControlMode.PercentOutput, rightSpeed);
    driveRightSpark.set(ControlMode.PercentOutput, rightSpeed);
  }

  @Override
  public void autonomousInit() {
    // Add any initialization code for autonomous mode here.
  }

  @Override
  public void autonomousPeriodic() {
    driveLeftSpark.set(ControlMode.PercentOutput, 1.0);
    driveRightSpark.set(ControlMode.PercentOutput, 1.0);
  }
}