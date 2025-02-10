package frc.robot.components;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class DriveTrain {
    SwerveModule module1;
    SwerveModule module2;
    SwerveModule module3;
    SwerveModule module4;
    double turn = 0;
    public DriveTrain(SwerveModule ... modules){
        module1 = modules[0];
        module2 = modules[1];
        module3 = modules[2];
        module4 = modules[3];
        
        //Ethan's code
        Modules = modules;
        kinematics = new SwerveDriveKinematics(
            module1.moduleLocation,
            module2.moduleLocation,
            module3.moduleLocation,
            module4.moduleLocation
        );
    }
    public void setDriveDirection(double angle){
        module1.TurnTo(angle+turn);
        module2.TurnTo(angle+turn);
        module3.TurnTo(angle-turn);
        module4.TurnTo(angle-turn);
    }
    public void setSpeed(double speed){
        module1.setDriveSpeed(speed);
        module2.setDriveSpeed(speed);
        module3.setDriveSpeed(speed);
        module4.setDriveSpeed(speed);
    }
    public void setTurn(double turn) {
        this.turn = turn;
    }
    
    /* 
     * ETHAN'S CODE
     */
    SwerveModule[] Modules;
    SwerveDriveKinematics kinematics;
    public void moveModules(SwerveModuleState... states){
        for (int i = 0; i < Modules.length; i++) {
            Modules[i].moveModule(states[i]);
        }
    }
    //lets make sure this works before more complicated stuff
    public void turnWheelsTo(double angleInDeg){
        SwerveModuleState pointState = new SwerveModuleState(
            0, 
            new Rotation2d(
                Units.degreesToRadians(angleInDeg)
            )
        );
        SwerveModuleState[] pointStates = {
            pointState, pointState, pointState, pointState
        };
        moveModules(pointStates);
    }
    
    public void setWheelVelocity(double percentOfMaxSpeed){
        percentOfMaxSpeed = Math.min(Math.max(percentOfMaxSpeed, -1), 1);
        percentOfMaxSpeed *= SetupConstants.kSpeedAt12VoltsMps;
        SwerveModuleState velState = new SwerveModuleState(
            percentOfMaxSpeed,
            new Rotation2d(0)
        );
        SwerveModuleState[] velStates = {
            velState, velState, velState, velState
        };
        moveModules(velStates);
    }
    
    public void useChassisSpeed(ChassisSpeeds speeds, Translation2d centerOfRot){
        
        moveModules(
            kinematics.toSwerveModuleStates(speeds, centerOfRot)
        );
    }
    
    public void useChassisSpeed(ChassisSpeeds speeds){
        useChassisSpeed(speeds, new Translation2d());
    }
    double maxRotationPerSec = 1;
    public void properSwerveRobotCentric(
        double percentXVel, 
        double percentYVel,
        double rotationSpeedRadPS
    ){
        percentXVel *= SetupConstants.kSpeedAt12VoltsMps;
        percentYVel *= SetupConstants.kSpeedAt12VoltsMps;
        //should limit RotationPS here/ account for limit
        rotationSpeedRadPS *= maxRotationPerSec;
        useChassisSpeed(
            new ChassisSpeeds(percentXVel, percentYVel, rotationSpeedRadPS)
        );
    }
    
}
