package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.SwerveUtils.RevMotor;
import frc.robot.SwerveUtils.RevMotor.RevMotorSetPosition;


public class Elevator implements Subsystem {
    
    // SparkMax upDownMotor = new SparkMax(0, MotorType.kBrushless);
    // SparkClosedLoopController upDownMotorCLContr = upDownMotor.getClosedLoopController();
    
    RevMotorSetPosition upDownMotor;
    RevMotorSetPosition armMotor;
    RevMotor coralIntakeMotor;
    
    //TODO get actual max rotation when build is done
    // double maxRot = 100;
    //TODO get percent height of levels wanted
    double l1 = 0;
    double l2 = 0.5;
    double l3 = 1;
    double[] levels = {l1, l2, l3};
    //arm positions, one for intake, one for output
    double[] armPositions = {0.2, 0.30};
    
    public Elevator(){
        SparkMaxConfig config = new SparkMaxConfig();
        // upDownMotorCLContr.setReference(0, SparkBase.ControlType.kPosition);
        //TODO, find device id
        upDownMotor = (RevMotorSetPosition) new RevMotorSetPosition(
            new SparkMax(0, MotorType.kBrushless),
             true,
             levels
        ).setMaxRot(100)//TODO get actual max rotation
        .setMinRot(1);
        
        config
            .inverted(true)
            .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(1)//keeping in rotations
            .velocityConversionFactor(1.0/60.0);// convert to rotations per second
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //TODO fix these
            .pid(1.0, 0.0, 0.0);
            
        upDownMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        SparkMaxConfig configArm = new SparkMaxConfig();
        //TODO, find device id
        armMotor = (RevMotorSetPosition) new RevMotorSetPosition(
            new SparkMax(1, MotorType.kBrushless),
             true,
             levels
        ).setMaxRot(100)//TODO get actual max rotation
        .setMinRot(1);
        
        config
            .inverted(false)
            .idleMode(IdleMode.kBrake);
        config.encoder
            .positionConversionFactor(1)//keeping in rotations
            .velocityConversionFactor(1.0/60.0);// convert to rotations per second
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            //TODO fix these
            .pid(1.0, 0.0, 0.0);
            
        armMotor.configure(configArm, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        coralIntakeMotor = new RevMotor(2, MotorType.kBrushless);
    }
    
    public void CoralIn(){
        coralIntakeMotor.setSpeed(0.5);
    }
    public void CoralOut(){
        coralIntakeMotor.setSpeed(0.5);
    }
    
    public void armIntake(){
        armMotor.goToSetPosition(1);
    }
    
    public void armOutput(){
        armMotor.goToSetPosition(2);
    }
    
    public void goToHeightPercent(double percentHeight){
        upDownMotor.goToRotationPercent(percentHeight);
    }
    
    // level must range between 1 and 3 inclusive
    public void goToLevel(int level){
        goToHeightPercent(levels[level-1]);
    }
    
    public void moveElavatorWithSpeed(double speed){
        upDownMotor.setSpeed(speed);
    }
    
    @Override
    public void periodic(){
        upDownMotor.resetReference();
        armMotor.resetReference();
        coralIntakeMotor.resetReference();
    }
    //Using run once here because the motor will continue to go to position/speed
    //it doesn't need to be called periodically, only when a change in motion is wanted
    public Command goToHeightPercentCommand(double percentHeight){
        return runOnce( ()-> {goToHeightPercent(percentHeight);});
    }
    
    public Command goToLevelCommand(int level){
        return runOnce( ()->{goToLevel(level);});
    }
    
    public Command moveElavatorWithSpeedCommand(double speed){
        return runOnce(()->{moveElavatorWithSpeed(speed);});
    }
    
    public Command goToLevel2AndOutputTest(){
        //test command chaining for outputting coral on level 2
        Command f = new Command() {
            @Override
            public void execute(){
                goToLevel(2);
            }
            
            @Override
            public boolean isFinished(){
                double elevatorRot = upDownMotor.Motor.getEncoder().getPosition();
                double wantedRot = upDownMotor.getRotationsFromPercent(l2);
                double withinRange = 0.2;
                if( 
                    elevatorRot > wantedRot-withinRange
                    && elevatorRot < wantedRot+withinRange
                ){
                    // within range, should end
                    return false;
                }
                return true;
            }
        };
        f.addRequirements(this);
        
        Command d = new Command() {
            @Override
            public void execute(){
                armOutput();
            }
            
            @Override
            public boolean isFinished(){
                double armRot = armMotor.Motor.getEncoder().getPosition();
                double wantedRot = armMotor.getRotationsFromPercent(l2);
                double withinRange = 0.2;
                if( 
                    armRot > wantedRot-withinRange
                    && armRot < wantedRot+withinRange
                ){
                    // within range, should end
                    return false;
                }
                return true;
            }
            
            @Override
            public void end(boolean interrupted){
                if(!interrupted){
                    CoralOut();
                }
            }
        };
        
        
        d.addRequirements(this);
        
        return f.andThen(d);
    }
    
    public Command RunCommandToDoThing(){
        return run(()->{
            //spin I guess
        });
    }
}
