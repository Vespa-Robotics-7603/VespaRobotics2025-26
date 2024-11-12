package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot
{
	private VictorSPX driveLeft1 = new VictorSPX(1);
	private VictorSPX driveRight1 = new VictorSPX(3);
	private VictorSPX driveLeft2 = new VictorSPX(2);
	private VictorSPX driveRight2 = new VictorSPX(4);
	private Joystick newJoystick = new Joystick(0);
	CANSparkMax climbMotor = new CANSparkMax(1, MotorType.kBrushless);
	CANSparkMax shooterMotor1 = new CANSparkMax(3, MotorType.kBrushed);
	CANSparkMax shooterMotor2 = new CANSparkMax(4, MotorType.kBrushed);

	Timer autoTimer = new Timer();

	public void driveRobot(double leftspeed, double rightspeed)
	{
		if (leftspeed > 1 || leftspeed < -1) 
			leftspeed = Math.signum(leftspeed);
		if (rightspeed > 1 || rightspeed < -1) 
			rightspeed = Math.signum(rightspeed);

		driveLeft1.set(ControlMode.PercentOutput, leftspeed);
		driveLeft2.set(ControlMode.PercentOutput, leftspeed);
		driveRight2.set(ControlMode.PercentOutput, rightspeed);
		driveRight1.set(ControlMode.PercentOutput, rightspeed);

		return;
	}

	public void disableAllMotors()
	{
		driveRobot(0, 0);
		shooterMotor1.set(0);
		shooterMotor2.set(0);
		climbMotor.set(0);

		return;
	}

	public int handleButtons()
	{
		double climbascentspeed = 1;
		double climbdescentspeed = 1;
		double intakespeed = 1;
		double firingspeed = 1;
		double fastturnspeed = 0.35;
		double slowturnspeed = 0.2;
		int[] climbbuttons = {11, 12};
		int[] shooterbuttons = {1, 2, 6};
		int[] movebuttons = {7, 8, 9, 10};
		int climbmode = 0;
		int shootermode = 0;
		int movemode = 0;
		int inhibitmovement = 1;
		int i;

		climbascentspeed *= -1;
		firingspeed *= -1;

		// Notes:
		//
		// climbmode += 1 * Math.pow(2, i) works in a similar way to that of a
		// binary number system, assigning a different "fingerprint" value for
		// every given combination of button presses that one can deploy when
		// one goes about using the controller to manipulate the robot's motors.
		//
		// The following code was specifically designed to be functionally
		// identical to the old code insofar as its functionality is concerned,
		// with the only exception to this rule being associated with that of 
		// the functionality of the new turn binds.

		for (i = 0; i < climbbuttons.length; i++) {
			if (newJoystick.getRawButton(climbbuttons[i]))
				climbmode += 1 * Math.pow(2, i);
		}

		switch (climbmode) {
		case 1:
		case 3:
			climbMotor.set(climbascentspeed);
			break;
		case 2:
			climbMotor.set(climbdescentspeed);
			break;
		default:
			climbMotor.set(0);
			break;
		}

		for (i = 0; i < shooterbuttons.length; i++) {
			if (newJoystick.getRawButton(shooterbuttons[i]))
				shootermode += 1 * Math.pow(2, i);
		}

		switch (shootermode) {
		case 1:
		case 3:
		case 5:
			shooterMotor1.set(firingspeed);
			shooterMotor2.set(firingspeed);
			break;
		case 2:
		case 6:
			shooterMotor1.set(intakespeed);
			shooterMotor2.set(intakespeed);
			break;
		case 4:
			shooterMotor2.set(firingspeed);
			break;
		default:
			shooterMotor1.set(0);
			shooterMotor2.set(0);
			break;
		}

		for (i = 0; i < movebuttons.length; i++) {
			if (newJoystick.getRawButton(movebuttons[i]))
				movemode += 1 * Math.pow(2, i);
		}

		// TODO: Fine-tune drive bind properties with drivers,
		// handle other button combinations in a graceful manner.

		switch (movemode) {
		case 1:
			driveRobot(-fastturnspeed, -fastturnspeed);
			break;
		case 2:
			driveRobot(fastturnspeed, fastturnspeed);
			break;
		case 4:
		case 5:
			driveRobot(-slowturnspeed, -slowturnspeed);
			break;
		case 8:
		case 10:
			driveRobot(slowturnspeed, slowturnspeed);
			break;
		default:
			inhibitmovement = 0;
			break;
		}

		return inhibitmovement;
	}

	@Override
	public void robotInit()
	{
		CameraServer.startAutomaticCapture();
	}

	@Override
	public void teleopInit()
	{
		disableAllMotors();
	}

	@Override
	public void teleopPeriodic()
	{
		double x_ampl = newJoystick.getRawAxis(1);
		double y_ampl = newJoystick.getRawAxis(0);
		double leftspeed = 0;
		double rightspeed = 0;
		double deadzone = 0.01;

		if (-deadzone < x_ampl && x_ampl < deadzone) x_ampl = 0;
		if (-deadzone < y_ampl && y_ampl < deadzone) y_ampl = 0;

		leftspeed = (y_ampl - x_ampl);
		rightspeed = (y_ampl + x_ampl);

		if (handleButtons() == 0) driveRobot(leftspeed, rightspeed);
	}

	@Override
	public void autonomousInit()
	{
		autoTimer.restart();
		autoTimer.start();
		disableAllMotors();
	}

	@Override
	public void autonomousPeriodic()
	{
		double[] timeincrements = {0.5, 0.5, 1, 1, 1, 3};
		double timeelapsed = autoTimer.get();
		double accumulator = 0;
		double enableshooter = 0;
		double jerkspeed = 0.5;
		double normalspeed = 0.25;
		int mode = 0;
		int stage = 0;
		int i;

		enableshooter *= -1;

		for (i = 0; i < timeincrements.length; i++) {
			accumulator += timeincrements[i];
			if (accumulator > timeelapsed) {
				stage = i;
				break;
			}
		}

		switch (stage) {
		case 0:
			driveRobot(-jerkspeed, jerkspeed);
			shooterMotor2.set(enableshooter);
			break;
		case 1:
			driveRobot(jerkspeed, -jerkspeed);
			break;
		case 2:
			driveRobot(0, 0);
			break;
		case 3:
			shooterMotor1.set(enableshooter);
			shooterMotor2.set(enableshooter);
			break;
		case 4:
			driveRobot(-normalspeed, normalspeed);
			shooterMotor1.set(0);
			shooterMotor2.set(0);
			break;
		default:
			disableAllMotors();
			break;
		}
	}
}
