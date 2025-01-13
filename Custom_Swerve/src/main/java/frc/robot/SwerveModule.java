package frc.robot;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.trajectory.TrapezoidProfile;




public class SwerveModule {
    private TalonFX driveMotor;
    private TalonFX turnMotor;
    private CANcoder coder;
    private double offset;
    Slot0Configs pidConfigs = new Slot0Configs();
    SwerveModule(TalonFX driveMotor, TalonFX turnMotor, CANcoder coder, double offset){
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.coder = coder;
        this.offset = offset;

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
        
    }
    public void setDriveSpeed(double speed){
        driveMotor.set(speed);
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
        TrapezoidProfile.State goal = new TrapezoidProfile.State(rotations/4 + offset,2);
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
}
