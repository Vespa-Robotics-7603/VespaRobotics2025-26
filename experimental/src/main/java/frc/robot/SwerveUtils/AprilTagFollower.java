package frc.robot.SwerveUtils;

import java.util.List;
import java.util.function.BooleanSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.TrajectoryFollower;
import frc.robot.subsystems.Vision;

public class AprilTagFollower {
    
    TrajectoryFollower follower;
    Vision photonVison;
    
    //This is so that we can stop reading/moving to april tags
    private boolean stopFollowing = false;
    
    public AprilTagFollower(Vision photonVision, TrajectoryFollower follower){
        this.follower = follower;
        this.photonVison = photonVision;
    }
    
    public SwerveControllerCommand makeFollowCommand(double XCoord, double YCoord, double Yaw){
        return follower.generateMovementCommand(
            follower.generateTrajectory(
                new TrajectoryTarget2d(XCoord, YCoord, Yaw)
            )
        );
    }
    
    public PhotonTrackedTarget scanFor(int id){
        List<PhotonTrackedTarget> targets = photonVison.getAllResults();
        for (PhotonTrackedTarget target : targets) {
            if(target.getFiducialId() == id) return target;
        }
        return null;
    }
    
    public void scheduleFollowCommand(int id, double offsetX, double offsetY, double offsetRAD, BooleanSupplier cancelCondition){
        PhotonTrackedTarget target = scanFor(id);
        if(target != null && !stopFollowing){
            //found, now follow
            Transform3d location = target.getBestCameraToTarget();
            
            SwerveControllerCommand followCommand = makeFollowCommand(
                location.getX() + offsetX,
                location.getY() + offsetY,
                location.getRotation().getZ() + offsetRAD
            );
            // both commands will run until one finishes, and waitUntil ends on command
            Command cancel = Commands.race(
                Commands.waitUntil(cancelCondition),
                followCommand
            );
            
            CommandScheduler.getInstance().schedule(cancel); 
            // we don't want to repeatedly schedule the same command
            // or multiple follow commands
            stopFollowing = true;
            System.out.println("SCHEDULED!");
        }
    }
    
    public Command alignWithTag(double seconds, int id, double offsetX, double offsetY, double offsetRAD, BooleanSupplier cancelCondition){
        return Commands.race(
            Commands.waitSeconds(seconds),
            // run until given time is up
            Commands.runEnd(
                ()->scheduleFollowCommand(id, offsetX, offsetY, offsetRAD, cancelCondition),
                ()->stopFollowing = false,
                photonVison
            )
        );
    }
    
    public Command alignWithTag(int id, double offsetX, double offsetY, double offsetRAD, BooleanSupplier cancelCondition){
        return alignWithTag(1, id, offsetX, offsetY, offsetRAD, cancelCondition);
    }
    
    public Command alignWithTag(double seconds, int id, double offsetX, double offsetY, double offsetRAD){
        return alignWithTag(seconds, id, offsetX, offsetY, offsetRAD, ()->false);
    }
    
    public Command alignWithTag(int id, double offsetX, double offsetY, double offsetRAD){
        return alignWithTag(id, offsetX, offsetY, offsetRAD, ()->false);
    }
    
    public Command alignWithTag(int id){
        return alignWithTag(id, 0, 0, 0);
    }


}
