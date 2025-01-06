package frc.robot.subsystems.mine;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveControlRequestParameters;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Custom swerve drivetrain by
 * <i>Ethan</i>
  */
public class SwerveEthan {
    
    public class OdometryEthan{
        SwerveDrivePoseEstimator poseEstimate;
        Supplier<Double> getCurYaw;
        Supplier<Double> getRateOfChangeYaw;
        Supplier<Double> getTimeSinceLastUpdate;

        public Pose2d updateDeg(SwerveModulePosition[] pos){
            //TODO
            double yaw = accountForLatency(
                getCurYaw.get(), 
                getRateOfChangeYaw.get(),
                getTimeSinceLastUpdate.get() 
            );
            return poseEstimate.update(Rotation2d.fromDegrees(yaw), pos);
        }

        public double accountForLatency(double yaw, double ROCYaw, double time){
            return yaw + (ROCYaw * time);
        }
    }
    /* 
     * So what do I need
     * - a function that applys to the swerve request
     * - a consumer for odomerty (no, I need it managed by me)
     * - a consumer for telemetry
     */

    SwerveDriveKinematics physicsMachine;
    SwerveRequest requestToUse;
    SwerveControlRequestParameters requestParams;
    SwerveModule[] theCornersAndStuff;
    SwerveModulePosition[] cornerPosMoved;
    SwerveModuleState[] cornerStates;
    Translation2d[] swervePositions;
    Rotation2d FieldForwrdDirection;
    Rotation2d DriverFwdRelToRedAllience;
    OdometryEthan poseStuff;

    public void moveTheBot(SwerveRequest useMe){
        if(useMe == null) useMe = requestToUse;
        //Update params
        for (int i = 0; i < theCornersAndStuff.length; ++i) {
            //TODO, decide weather to update here
            cornerPosMoved[i] = theCornersAndStuff[i].getPosition(false);
            cornerStates[i] = theCornersAndStuff[i].getCurrentState();
        }
        poseStuff.updateDeg(cornerPosMoved);

        ChassisSpeeds curSpeeds = physicsMachine.toChassisSpeeds(cornerStates);

        requestParams.currentPose = 
            poseStuff.poseEstimate.getEstimatedPosition()
            .relativeTo(new Pose2d(0, 0, FieldForwrdDirection));
        requestParams.kinematics = physicsMachine;
        requestParams.swervePositions = swervePositions;
        requestParams.currentChassisSpeed = curSpeeds;
        requestParams.timestamp = 0;//TODO
        requestParams.updatePeriod = 0;//TODO (also doesn't change)
        requestParams.operatorForwardDirection = DriverFwdRelToRedAllience;

        useMe.apply(requestParams, theCornersAndStuff);

        // then do telemetry stuff

    }
}
