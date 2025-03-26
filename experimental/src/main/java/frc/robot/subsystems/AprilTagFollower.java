package frc.robot.subsystems;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.SwerveUtils.TrajectoryTarget2d;

public class AprilTagFollower extends Command {
    
    /* 
     * Should this be a subsystem?
     * I don't want it to be...
     * How would this work?
     * periodically monitor the apriltags
     * then make/schedule commands accordingly
     * WAIT, make this a command!
     */
    
     //TODO, get drivetrain
    TrajectoryFollower follower = new TrajectoryFollower(null);
    Vision photonVison = new Vision(null);
    
    int currentFollowId = -1;
    Command currentFollowCommand =null;
    
    //This is so that we can stop reading/moving to april tags
    boolean stopFollowing = false;
    
    public SwerveControllerCommand makeFollowCommand(Transform3d coords){
        return follower.generateMovementCommand(
            follower.generateTrajectory(
                new TrajectoryTarget2d(coords.getX(), coords.getY(), coords.getRotation().getZ())
            )
        );
    }
    
    @Override
    public void execute(){
        List<PhotonTrackedTarget> targets = photonVison.getAllResults();
        for (PhotonTrackedTarget target : targets) {
            target.getFiducialId();
            //TODO decide which to follow
        }
    }
    
    @Override
    public boolean isFinished(){
        return stopFollowing;
    }
}
