package frc.robot.SwerveUtils;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * Encasulaes methods and variable needed to control the revlib motors.
 * <p>
 * <b>NOTE:</b> motors are not automaticaly configured properly.
 * <br>
 * You will need to configure the motor yourself by:
 * </p>
 * <ul>
 *      <li>Passing the configured motor to the constuctor {@link #RevMotor(SparkMax, boolean)};</li>
 *      <li>Confuguring the motor later by acessing the {@link #Motor} field;
 *      <li>Or by using the {@link #configure(SparkMaxConfig, ResetMode, PersistMode)} method.
 * </ul>
  */
public class RevMotor {
    
    public SparkMax Motor;
    
    public SparkClosedLoopController CLController;
    
    protected double maxRot = 10;

    protected double minRot = 1;

    protected double minRotOffset = 0.1;
    
    protected double maxRotOffset = 0.1;
    

    private double refVal = 0;
    private ControlType controlT = ControlType.kPosition;
    
    public RevMotor(int deviceId, MotorType motorType){
        this(new SparkMax(deviceId, motorType), false);
        
    }
    
    public RevMotor(SparkMax motor, boolean IsAlreadyConfigured){
        Motor = motor;
        CLController = Motor.getClosedLoopController();
        
        SparkMaxConfig config = new SparkMaxConfig();
        CLController.setReference(refVal, controlT);
        
        if(!IsAlreadyConfigured){
            config
                .inverted(false)
                .idleMode(IdleMode.kBrake);
            config.encoder
                .positionConversionFactor(1)//keeping in rotations
                .velocityConversionFactor(1);//Keep in rotaion per minute (ew) by default
            config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
                // .pid(1.0, 0.0, 0.0);
            Motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
        
    }
    
    public REVLibError configure(SparkMaxConfig configs, ResetMode resetMode, PersistMode persistMode){
       return Motor.configure(configs, resetMode, persistMode);
    }
    
    /**
     * Sets the reference of the {@link #CLController}. Should be called periodically.
     * Ensures the motor has not passed the min or max rotations.
      */
    public void resetReference(){
        //to prevent to motor from turning to far down
        double motPos = Motor.getEncoder().getPosition();
        if(motPos <= minRot){
            refVal = minRot + minRotOffset;
            controlT = ControlType.kPosition;
        }
        else if(motPos >= maxRot){
            refVal = maxRot - maxRotOffset;
            controlT = ControlType.kPosition;
        }
        defaultSetRef();
    }
    
    /**
     * Here in case a more complicated way of setting references is needed, 
     * because accessing {@link #CLController} directly to set reference might be overridin
     * when {@link #resetReference()} is called. In short, this allows the other 
     * {@link SparkClosedLoopController#setReference(double, ControlType)} methods to be used.
     * @see SparkClosedLoopController#setReference(double, ControlType, com.revrobotics.spark.ClosedLoopSlot)
     * @see SparkClosedLoopController#setReference(double, ControlType, com.revrobotics.spark.ClosedLoopSlot, double)
     * @see SparkClosedLoopController#setReference(double, ControlType, com.revrobotics.spark.ClosedLoopSlot, double, com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits)
     * @return The {@code REVLibError} normaly returned by {@link SparkClosedLoopController#setReference(double, ControlType)}
      */
    protected REVLibError defaultSetRef(){
        return CLController.setReference(refVal, controlT);
    }
    
    public REVLibError setRefernce(double value, ControlType type){
        return CLController.setReference(value, type);
    }
    
    public double getRotationsFromPercent(double percent){
        return (maxRot-minRot) *percent+ minRot;
    }
    
    /**
     * Sets speed to value between -1 and 1
     * @param speed the speed as a percent from -1 to 1
      */
    public void setSpeed(double speed){
        controlT = ControlType.kDutyCycle;
        refVal = Math.min(Math.max(speed, -1), 1);
    }
    
    public void goToMaxRotationPercent(double percentMaxRotation){
        refVal = (maxRot-minRot) * Math.min(Math.max(percentMaxRotation, 0), 1) + minRot;
        controlT = ControlType.kPosition;
    }
    
    public void goToRotation(double rotations){
        refVal = Math.min(Math.max(rotations, minRot), maxRot);
        controlT = ControlType.kPosition;
    }
    
    public double getMaxRot() {
        return maxRot;
    }

    public RevMotor setMaxRot(double newMaxRot) {
        maxRot = Math.max(newMaxRot, minRot+minRotOffset);
        return this;
    }
    
    public double getMinRot() {
        return minRot;
    }

    public RevMotor setMinRot(double newMinRot) {
        minRot = Math.min(maxRot, newMinRot);
        return this;
    }
    
    public double getMinRotOffset() {
        return minRotOffset;
    }

    public RevMotor setMinRotOffset(double newMinRotOff) {
        minRotOffset = newMinRotOff;
        return this;
    }
    
    public double getMaxRotOffset() {
        return maxRotOffset;
    }

    public RevMotor setMaxRotOffset(double newMaxRotOffset) {
        maxRotOffset = newMaxRotOffset;
        return this;
    }
    
    public static class RevMotorSetPosition extends RevMotor{
        double[] setPositionsPercent;
        public RevMotorSetPosition(int deviceId, MotorType type, double... percentOfPositions){
            super(deviceId, type);
            setPositionsPercent = new double[percentOfPositions.length];
            for (int i = 0; i < percentOfPositions.length; i++) {
                setPositionsPercent[i] = Math.min(Math.max(percentOfPositions[i], 0), 1);
            }
        }
        public RevMotorSetPosition(SparkMax motor, boolean IsAlreadyConfigured, double... percentOfPositions){
            super(motor, IsAlreadyConfigured);
            setPositionsPercent = new double[percentOfPositions.length];
            for (int i = 0; i < percentOfPositions.length; i++) {
                setPositionsPercent[i] = Math.min(Math.max(percentOfPositions[i], 0), 1);
            }
        }
    
        public void goToSetPosition(int positionNumber){
            if(positionNumber >= setPositionsPercent.length || positionNumber < 0) 
                positionNumber=0;
            goToMaxRotationPercent(setPositionsPercent[positionNumber-1]);
        }
    }
}