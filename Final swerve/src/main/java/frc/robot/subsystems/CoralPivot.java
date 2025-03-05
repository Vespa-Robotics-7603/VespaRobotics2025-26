package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Subsystem;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.SwerveUtils.RevMotor.RevMotorSetPosition;

public class CoralPivot implements Subsystem{
    
    
    RevMotorSetPosition armMotor;
    //arm positions, one for intake, one for output
    double[] armPositions = {0.2, 0.30};
    
    public CoralPivot(){
        SparkMaxConfig configArm = new SparkMaxConfig();
        
        armMotor = (RevMotorSetPosition) new RevMotorSetPosition(
            new SparkMax(3, MotorType.kBrushless),
             true,
             armPositions
        ).setMaxRot(100)//TODO get actual max rotation
        .setMinRot(1);
        
        configArm
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        configArm.encoder
            .positionConversionFactor(1)//keeping in rotations
            .velocityConversionFactor(1.0/60.0);// convert to rotations per second
        configArm.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //TODO fix these
            .pid(1.0, 0.0, 0.0);
            
        armMotor.configure(configArm, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }
    
    public void armIntake(){
        armMotor.goToSetPosition(1);
    }
    
    public void armOutput(int level){
        armMotor.goToSetPosition(2);
    }
    
    @Override
    public void periodic(){
        armMotor.resetReference();
    }
}
