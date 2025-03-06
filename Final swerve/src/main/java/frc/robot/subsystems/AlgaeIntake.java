package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.SwerveUtils.RevMotor;

public class AlgaeIntake implements Subsystem {
    RevMotor algeaMotor;
    
    double algaeInSpeed = 0.3;
    double algaeOutSpeed =-0.3;
    
    public AlgaeIntake(){
        
        algeaMotor = new RevMotor(4, MotorType.kBrushless);
    }
    
    public void AlgaeIn(){
        algeaMotor.setSpeed(algaeInSpeed);
    }
    public void AlgaeOut(){
        algeaMotor.setSpeed(algaeOutSpeed);
    }
    public void Stop(){
        algeaMotor.setSpeed(0);
    }
    
    @Override
    public void periodic(){
        algeaMotor.resetReference();
    }
    
    public Command AlgaeInCommand(){
        return runOnce(this::AlgaeIn);
    }
    
    public Command AlgaeOutCommand(){
        return runOnce(this::AlgaeOut);
    }
    
    public Command AlgaeStopCommand(){
        return runOnce(this::Stop);
    }
}
