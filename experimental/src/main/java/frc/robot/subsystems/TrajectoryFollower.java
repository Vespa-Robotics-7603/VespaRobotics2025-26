package frc.robot.subsystems;


import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.SwerveUtils.TrajectoryTarget2d;
import static frc.robot.constants.TrajectoryConstants.*;

import java.util.function.Supplier;

// TODO: Document and test everything, refactor if necessary.
public class TrajectoryFollower {
    
    // chat is this a pi reference? chat what comes after 3.141592653589793238462643383279502884197169399375105820974944592307816406286208998628034825342117067982148086513282306647093844609550582231725359408128481117450284102701938521105559644622948954930381964428810975665933446128475648233786783165271201909145648566923460348610454326648213393607260249141273724587006606315588174881520920962829254091715364367892590360011330530548820466521384146951941511609433057270365759591953092186117381932611793105118548074462379962749567351885752724891227938183011949129833673362440656643086021394946395224737190702179860943702770539217176293176752384674818467669405132000568127145263560827785771342757789609173637178721468440901224953430146549585371050792279689258923542019956112129021960864034418159813629774771309960518707211349999998372978049951059731732816096318595024459455346908302642522308253344685035261931188171010003137838752886587533208381420617177669147303598253490428755468731159562863882353787593751957781857780532171226806613001927876611195909216420198938095257201065485863278865936153381827968230301952035301852968995773622599413891249721775283479131515574857242454150695950829533116861727855889075098381754637464939319255060400927701671139009848824012858361603563707660104710181942955596198946767837449448255379774726847104047534646208046684259069491293313677028989152104752162056966024058038150193511253382430035587640247496473263914199272604269922796782354781636009341721641219924586315030286182974555706749838505494588586926995690927210797509302955321165
    // the answer is 3 by the way -ethan
    CommandSwerveDrivetrain drivetrain;
    SwerveDriveKinematicsConstraint constraint;
    HolonomicDriveController controller;

    // TODO: Determine our robot's ideal max speed.
    // NOTE: a billion dollars is not a valid answer.
    public TrajectoryFollower(CommandSwerveDrivetrain drivetrain){
        this.drivetrain = drivetrain;
        constraint = new SwerveDriveKinematicsConstraint(
            drivetrain.getKinematics(),
            MAX_SPEED
        );
        // Create a voltage constraint to ensure we don't accelerate too fast
        controller = new HolonomicDriveController(
            new PIDController(KP_X, KI_X, KD_X), 
            new PIDController(KP_Y, KI_Y, KD_Y),
            new ProfiledPIDController(KP_THETA, KI_THETA, KD_THETA,
                new TrapezoidProfile.Constraints(MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_ACCELERATION)
            )
        );
    }

    // TODO: Reformat this code so that it's more readable.
    /**
     * Generates a {@link TrajectoryConfig} for the robot
     * @param speed max speed in m/s
     * @param acceleration max acceleration in m/s/s
     * @return the config
      */
    public TrajectoryConfig generateTrajectoryConfig(double speed, double acceleration) {
        // Create config for trajectory
        return new TrajectoryConfig(
                speed,
                acceleration
            )
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(drivetrain.getKinematics())
            // Apply the voltage constraint
            .addConstraint(constraint);
    }

    /**
     * Generates a trajectory with given max speed, acceleration and points
     * @param speed max speed in m/s
     * @param acceleration max acceleration in m/s/s
     * @param target The start, middle, and end point as a 
     * {link TrajectoryTarget2d}
     * @return The wanted trajectory
      */
    public Trajectory generateTrajectory(double speed, double acceleration, TrajectoryTarget2d target) {
        TrajectoryConfig config = generateTrajectoryConfig(speed, acceleration);
        return TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0,0, new Rotation2d(0)),
            target.getWaypoints(),
            new Pose2d(target.getTranslation(), target.getRotation()),
            config
        );
    }
    /**
     * Generates a trajectory with the given points
     * @param target The start, middle, and end point as a 
     * {link TrajectoryTarget2d}
     * @return The wanted trajectory
      */
    public Trajectory generateTrajectory(TrajectoryTarget2d target){
        return generateTrajectory(MAX_SPEED, MAX_ACCELERATION, target);
    }
    
    public Command wrapGeneratedCommand(SwerveControllerCommand movementCommand){
        Supplier<Pose2d> oldPose = new Supplier<Pose2d>() {
            Pose2d oldValue = null;
            @Override
            public Pose2d get() {
                if (oldValue == null) {
                    //get pose
                    oldValue = drivetrain.getPose();
                    return oldValue;
                }
                else{
                    //already run, so return old value
                    Pose2d ret = oldValue;
                    oldValue = null;
                    return ret;
                }
                //this should act like a flip flop gate, switching from collecting pose, to giving it
                //I hope this works
            }
            
        };
        return Commands.sequence(
            Commands.runOnce(()->oldPose.get(), drivetrain), //first get run, should store the pose
            movementCommand,
            Commands.runOnce(()->{
                Pose2d oldPose2d = oldPose.get() //second call, should return stored value
                    .relativeTo(drivetrain.getPose())//this should give back an accurate pose
                ;
                drivetrain.resetPose(oldPose2d);
            }, drivetrain)
        );
        
    }
    
    public Command wrapGeneratedCommand(TrajectoryTarget2d target){
        return wrapGeneratedCommand(generateMovementCommand(target));
    }
    
    /**
     * Generates the command that moves along the given trajectory
     * @param trajectory the trajectory to follow
     * @return The command to move the robot
      */
    public SwerveControllerCommand generateMovementCommand(Trajectory trajectory) {
        return new SwerveControllerCommand(
            trajectory,
            drivetrain::getPose, 
            drivetrain.getKinematics(), 
            controller,
            this::moveWithState, 
            drivetrain
        );
    }

    /**
     * Generates the command that moves with the max speed, acceleration and points
     * @param speed max speed in m/s
     * @param acceleration max acceleration in m/s/s
     * @param target The start, middle, and end point as a 
     * {link TrajectoryTarget2d}
     * @return The command to move the robot
      */
    public SwerveControllerCommand generateMovementCommand(double speed, double acceleration, TrajectoryTarget2d target){
        Trajectory trajectory = generateTrajectory(speed, acceleration, target);
        SwerveControllerCommand command = generateMovementCommand(trajectory);
        return command;
    }
    
    /**
     * Generates the command that moves with the given points
     * @param target The start, middle, and end point as a 
     * {link TrajectoryTarget2d}
     * @return The command to move the robot
      */
    public SwerveControllerCommand generateMovementCommand(TrajectoryTarget2d target){
        return generateMovementCommand(generateTrajectory(target));
    }
    
    private void moveWithState(SwerveModuleState... states){
        // System.out.println("Applying speeds...");
        drivetrain.setControl(
            new SwerveRequest.ApplyRobotSpeeds().withSpeeds(
                drivetrain.getKinematics().toChassisSpeeds(states)
            )
        );
    }
}
