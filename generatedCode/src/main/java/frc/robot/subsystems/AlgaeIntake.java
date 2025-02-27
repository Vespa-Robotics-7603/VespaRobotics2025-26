package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.SwerveUtils.RevMotor;

public class AlgaeIntake implements Subsystem {
    RevMotor algeaMotor;
    
    public AlgaeIntake(){
        
        algeaMotor = new RevMotor(5, MotorType.kBrushless);
    }
    
    public void AlgaeIn(){
        algeaMotor.setSpeed(0.3);
    }
    public void AlgaeOut(){
        algeaMotor.setSpeed(-0.3);
    }
    
    @Override
    public void periodic(){
        algeaMotor.resetReference();
    }
}
