package frc.robot.components;
import java.util.function.Function;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModuleConstants;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class SwerveModule {
    private TalonFX driveMotor;
    private TalonFX turnMotor;
    private CANcoder coder;
    private double direction;
    Slot0Configs pidConfigs = new Slot0Configs();
    public SwerveModule(
        TalonFX driveMotor, 
        TalonFX turnMotor, 
        CANcoder coder, 
        double direction, 
        LegacySwerveModuleConstants constants
    ){
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.coder = coder;
        this.direction = direction;

        //These slot configs are copied from the pheonix 6 documentation and should be
        //Accurate for talon motors, although we could recalculate if we want
        pidConfigs.kS = 0.25; // Add 0.25 V output to overcome static friction
        pidConfigs.kV = 0.5; // A velocity target of 1 rps results in 0.12 V output
        pidConfigs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        pidConfigs.kP = 10; // An error of 1 rps results in 0.11 V output
        pidConfigs.kI = 0; // no output for integrated error
        pidConfigs.kD = 0.2; // no output for error derivative


        turnMotor.getConfigurator().apply(pidConfigs);
        driveMotor.getConfigurator().apply(pidConfigs);
        
        /* 
         * ETHAN'S CODE
         */
        
        MMVControlRequest = new MotionMagicVoltage(0);
        VVControlRequest = new VelocityVoltage(0);
        //Don't know if these are useful
        // MMVControlRequest.UpdateFreqHz = 0;
        // VVControlRequest.UpdateFreqHz = 0;
        
        turnRequester = (doubleInput)->{
            return MMVControlRequest.withPosition(doubleInput);
        };
        driveRequester = (doubleInput)->{
            return VVControlRequest.withVelocity(doubleInput);
        };
        
        // I don't understand this part... but here it is
        
        //########### TURN MOTOR CONFIG #############
        
        // LegacySwerveModuleConstants
        //     constants = SetupContants.FrontLeft;
        TalonFXConfiguration steerConfigs = constants.SteerMotorInitialConfigs;
        steerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        steerConfigs.Slot0 = constants.SteerMotorGains;
        // Modify configuration to use remote CANcoder fused
        steerConfigs.Feedback.FeedbackRemoteSensorID = constants.CANcoderId;
        switch (constants.FeedbackSource) {
            case RemoteCANcoder:
                steerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
                break;
            case FusedCANcoder:
                steerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
                break;
            case SyncCANcoder:
                steerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
                break;
        }
        steerConfigs.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;

        steerConfigs.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
        steerConfigs.MotionMagic.MotionMagicAcceleration = steerConfigs.MotionMagic.MotionMagicCruiseVelocity / 0.100;
        steerConfigs.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
        steerConfigs.MotionMagic.MotionMagicExpo_kA = 0.1;

        steerConfigs.ClosedLoopGeneral.ContinuousWrap = true; // Enable continuous wrap for swerve modules

        steerConfigs.MotorOutput.Inverted = constants.SteerMotorInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        turnMotor.getConfigurator().apply(steerConfigs);
        
        //############### DRIVE MOTOR CONFIG ################
        TalonFXConfiguration driveConfigs = constants.DriveMotorInitialConfigs;
        driveConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        driveConfigs.Slot0 = constants.DriveMotorGains;
        driveConfigs.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
        driveConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
        driveConfigs.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
        driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        driveConfigs.MotorOutput.Inverted = constants.DriveMotorInverted 
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        driveMotor.getConfigurator().apply(driveConfigs);
        
        //############### CAN CODER CONFIG #################
        CANcoderConfiguration cancoderConfigs = constants.CANcoderInitialConfigs;
        cancoderConfigs.MagnetSensor.MagnetOffset = constants.CANcoderOffset;
        coder.getConfigurator().apply(cancoderConfigs);
        
        steerPosition = turnMotor.getPosition().clone();
        steerVelocity = turnMotor.getVelocity().clone();
        drivePosition = driveMotor.getPosition().clone();
        driveVelocity = driveMotor.getVelocity().clone();
        
        
        double rotationsPerWheelRotation = constants.DriveMotorGearRatio;
        double metersPerWheelRotation = 2 * Math.PI * Units.inchesToMeters(constants.WheelRadius);
        rotationPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;
        couplingRatioDriveRotorToCANcoder = constants.CouplingGearRatio;
        
        moduleLocation = new Translation2d(
            constants.LocationX,
            constants.LocationY
        );
    }
    public void setDriveSpeed(double speed){
        driveMotor.set(speed*direction);
    }
    public void setTurnSpeed(double speed){
        //This is deprecated and should only be used for testing
        turnMotor.set(speed);
    }
    public void TurnTo(Double rotations){
        //Trapaziod profile calculates the place on the motor to which you must turn
        //and turns to motor too it
        TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(100, 200));
        //create the state in which you want it to end up
        TrapezoidProfile.State goal = new TrapezoidProfile.State(rotations/4 + Constants.OFFSET,2);
        //create the state in which it is currently
        TrapezoidProfile.State setPoint = new TrapezoidProfile.State();
        //calculate how to get from the current state to the final goal state
        setPoint = profile.calculate(10, setPoint, goal);
        //Set requests to the motor
        PositionVoltage request = new PositionVoltage(0).withSlot(0);
        request.Velocity = setPoint.velocity;
        request.Position = setPoint.position;
        turnMotor.setControl(request);
        System.out.println("Ran TurnBy");
    }
    public CANcoder getCoder() {
        return coder;
    }
    
    /* 
     * ETHAN'S CHANGES 
     */
    Function<Double, ControlRequest> turnRequester;
    //needed for odometry and swerve
    SwerveModulePosition curSwervePos = new SwerveModulePosition();
    double rotationPerMeter;//TODOne get proper number
    StatusSignal<Angle> steerPosition;
    StatusSignal<AngularVelocity> steerVelocity;
    /** TODOne this is a weird number I don't understand */
    double couplingRatioDriveRotorToCANcoder ;
    Function<Double, ControlRequest> driveRequester;
    /** temp-ish, just seeing if this works */
    MotionMagicVoltage MMVControlRequest;
    /** temp-ish, just seeing if this works <br></br>
     * open loop voltage was used by the pre-made swerve modules, but this seems better
     */
    VelocityVoltage VVControlRequest;
    Translation2d moduleLocation;
    
    StatusSignal<Angle> drivePosition;
    StatusSignal<AngularVelocity> driveVelocity;
    
    public void moveModule(SwerveModuleState wantedState){
        
        updatePosition();
        wantedState.optimize(curSwervePos.angle);
        double rotationsToSet = wantedState.angle.getRotations();
        turnMotor.setControl(turnRequester.apply(rotationsToSet));
        
        double velocityForDrive = wantedState.speedMetersPerSecond * rotationPerMeter;
        double rotationError = rotationsToSet - steerPosition.getValueAsDouble();
        double cosScalar = Math.max(
            Math.cos(Units.rotationsToRadians(rotationError)),
            0
        );
        velocityForDrive *= cosScalar;
        double importantNumberVelocity = steerVelocity.getValueAsDouble();
        velocityForDrive += importantNumberVelocity * couplingRatioDriveRotorToCANcoder;
        
        driveMotor.setControl(driveRequester.apply(velocityForDrive));
    }
    public SwerveModulePosition updatePosition(){
        
        // Double drive_rot = BaseStatusSignal.getLatencyCompensatedValue(
        //     drivePosition.getValueAsDouble(), 
        //     driveVelocity.getValueAsDouble()
        // );
        // Measure<AngleUnit> angle_rot = BaseStatusSignal.getLatencyCompensatedValue(steerPosition, steerVelocity);
        
        drivePosition.refresh();
        driveVelocity.refresh();
        steerPosition.refresh();
        steerVelocity.refresh();
        
        double driveRot = compensateForLatency(
            drivePosition.getValueAsDouble(), 
            driveVelocity.getValueAsDouble(),
            drivePosition.getTimestamp().getLatency()
        );
        double angleRot = compensateForLatency(
            steerPosition.getValueAsDouble(), 
            steerVelocity.getValueAsDouble(),
            steerPosition.getTimestamp().getLatency()
        );
        
        driveRot -= angleRot * couplingRatioDriveRotorToCANcoder;
        curSwervePos.distanceMeters = driveRot / rotationPerMeter;
        curSwervePos.angle = Rotation2d.fromRotations(angleRot);
        return curSwervePos;
    }
    private double compensateForLatency(double val, double valPerSec, double timeSince){
        return val + (valPerSec * timeSince);
    }
    public TalonFX getDriveMotor(){
        return driveMotor;
    }
    public TalonFX getTurnMotor(){
        return turnMotor;
    }
}
