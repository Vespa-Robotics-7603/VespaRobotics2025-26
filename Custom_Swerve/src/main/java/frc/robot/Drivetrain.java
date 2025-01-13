package frc.robot;

public class Drivetrain {
    SwerveModule module1;
    SwerveModule module2;
    SwerveModule module3;
    SwerveModule module4;
    Drivetrain(SwerveModule ... modules){
        module1 = modules[0];
        module2 = modules[2];
        module3 = modules[3];
        module4 = modules[4];
    }
    public void turnTo(double angle){
        module1.TurnTo(angle);
        module2.TurnTo(angle);
        module3.TurnTo(angle);
        module4.TurnTo(angle);
    }
    public void setSpeed(double speed){
        module1.setDriveSpeed(speed);
        module2.setDriveSpeed(speed);
        module3.setDriveSpeed(speed);
        module4.setDriveSpeed(speed);
    }
}
