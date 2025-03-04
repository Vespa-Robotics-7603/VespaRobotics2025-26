package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

//this gives me an error so I can't look at it's methods, but it builds
import com.ctre.phoenix.motorcontrol.can.VictorSPX;


public class CoralIntake implements Subsystem {
    
    VictorSPX intakeMotor = new VictorSPX(1);
    
    public CoralIntake(){
        //TODO setup victor spx
    }
    
    public void CoralIn(){
        // coralIntakeMotor.moveByRotations(-1);
    }
    public void CoralOut(){
        // coralIntakeMotor.setSpeed(0.6);
    }
}
