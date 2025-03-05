package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

//this gives me an error so I can't look at it's methods, but it builds
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;

import com.ctre.phoenix.motorcontrol.ControlMode;


public class CoralIntake implements Subsystem {
    
    VictorSPX intakeMotor = new VictorSPX(9);
    
    public CoralIntake(){
        //TODO setup victor spx
        //setting pids (slot id, value)
        intakeMotor.config_kP(0,0.1);
        intakeMotor.config_kI(0, 0.1);
        intakeMotor.config_kD(0,0.1);
        //feed forwards I assume
        intakeMotor.config_kF(0, 0);
        //velocity in sensor units per 100 ms???? ew
        // intakeMotor.configMotionCruiseVelocity(2);
        //for all configs
        VictorSPXConfiguration conf = new VictorSPXConfiguration();
        // conf.slot0.kP = 0; //I don't think this should be used
        // conf.primaryPID. // nor should this
        // driveLeft1.configAllSettings(conf);
    }
    
    public void CoralIn(){
        // coralIntakeMotor.moveByRotations(-1);
        double pos = intakeMotor.getSelectedSensorPosition();
        //above probably gets position, but there is no documentation...
        double diffInRot = -0.3;
        intakeMotor.set(ControlMode.Position, pos + diffInRot);
    }
    public void CoralOut(){
        // coralIntakeMotor.setSpeed(0.6);
        intakeMotor.set(ControlMode.PercentOutput, 0.4);
    }
}
