package frc.robot;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;


import static edu.wpi.first.units.Units.Degree;


public class SwerveModule {
    TalonFX driveMotor;
    TalonFX turnMotor;
    CANcoder coder;
    double offset;
    SwerveModule(TalonFX driveMotor, TalonFX turnMotor, CANcoder coder){
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.coder = coder;
        this.offset = coder.getPosition().getValue().abs(Degree);
        System.out.println(offset);
    }
    public void setDriveSpeed(double speed){
        driveMotor.set(speed);
    }
    public void setTurnSpeed(double speed){
        turnMotor.set(speed);
    }
    public void setOffset(double offset) {
        this.offset = offset;
    }
    public double getOffset() {
        return offset;
    }
    public void TurnTo(int Degrees){
        double speed;
        if(Degrees>1){
            speed = 0.1;
        } else {
            speed = -0.1;
        }
        double targetTurn = offset + Degrees;
        System.out.println(targetTurn);
        System.out.println(coder.getPosition().getValue().in(Degree));
        if(coder.getPosition().getValue().in(Degree) == targetTurn){
            turnMotor.set(0);
        } else {
            turnMotor.set(speed);
        }
    }
    
}
