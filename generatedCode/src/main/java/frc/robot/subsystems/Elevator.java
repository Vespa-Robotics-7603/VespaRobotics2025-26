package frc.robot.subsystems;

import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Elevator implements Subsystem {
    
    //TODO, find device id
    SparkMax d = new SparkMax(0, MotorType.kBrushless);
    SparkClosedLoopController c = d.getClosedLoopController();
    
    //TODO get actual max rotation when build is done
    double maxRot = 100;
    
    public Elevator(){
        SparkMaxConfig config = new SparkMaxConfig();
        c.setReference(0, SparkBase.ControlType.kPosition);
        
        config
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(1000)
            .velocityConversionFactor(1000);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(1.0, 0.0, 0.0);
            
        d.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    
    public void motor(){
        
    }
    
    public Command RunCommandToDoThing(){
        return run(()->{
            //spin I guess
        });
    }
}
