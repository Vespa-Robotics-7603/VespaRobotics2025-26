package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionData.RobotPose;
import frc.robot.subsystems.VisionData.VisionTarget;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;

// TODO: (maybe) refactor the code found here
public class Vision extends SubsystemBase {
    private PhotonCamera camera;
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final CommandSwerveDrivetrain drivetrain;
    private boolean currentCam = true; // true means top camera, false means bottom camera
    private final PhotonCamera topCam = new PhotonCamera("topCam");
    private final PhotonCamera bottomCam = new PhotonCamera("bottomCam");
    // private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
    private final SwerveRequest.FieldCentricFacingAngle snapTo = new SwerveRequest
        .FieldCentricFacingAngle()
        .withDeadband(MaxSpeed*0.035)
        .withDriveRequestType(DriveRequestType.Velocity);

    PhoenixPIDController turnPID = new PhoenixPIDController(0, 0, 0);
    PhoenixPIDController drivePID = new PhoenixPIDController(0, 0, 0);
    Timer tmr = new Timer();
    
    //the results should be updated once per loop, so it's being updated in periodic
    private List<PhotonPipelineResult> latestResults= List.of();

    public PhotonTrackedTarget getBestTarget() {
        // TO DONE: Replace method with something that isn't deprecated
        // PhotonPipelineResult result = camera.getLatestResult();
        // List<PhotonPipelineResult> result = getAllResult();
        //TODO, refactor for intended/better usage, instead of limiting ourselves to 1 result
        return (!latestResults.isEmpty()) ? latestResults.get(0).getBestTarget() : null;
    }
    
    public List<PhotonTrackedTarget> getAllResults(){
        return latestResults.get(0).getTargets();
    }

    public Transform3d getTargetTransform() {
        PhotonTrackedTarget target = getBestTarget();
        return (target != null) ? target.getBestCameraToTarget() : null;
    }

    public double getTargetYaw() {
        PhotonTrackedTarget target = getBestTarget();
        return (target != null) ? target.getYaw() : 0.0;
    }

    public double getTargetDistanceX() {
        PhotonTrackedTarget target = getBestTarget();
        return (target != null) ? target.getBestCameraToTarget().getX() : -1;
    }
    
    public double getTargetDistanceY() {
        PhotonTrackedTarget target = getBestTarget();
        return (target != null) ? target.getBestCameraToTarget().getY() : -1;
    }

    public void updateTargetPose(VisionTarget targetpose) {
        targetpose.yaw = getTargetYaw();
        targetpose.x = getTargetDistanceX();
        targetpose.y = getTargetDistanceY();
    }

    public void updateRobotPose(RobotPose robotpose) {
        robotpose.yaw = drivetrain.getState().Pose.getRotation().getRadians();
        robotpose.x = drivetrain.getState().Pose.getX();
        robotpose.y = drivetrain.getState().Pose.getY();
    }

    // TODO: maybe split off pure vision info code from april tag code
    public void moveToTarget(double xoffset, double yoffset, double rotationoffset) {
        
    }

    // TODO: Write proper vision code
    /*
    public void followAprilTag(){
        var target = this.getBestTarget();
    
        if (target != null) {
            double targetYaw = this.getTargetYaw(); // Degrees
            double xPose = this.getTargetDistanceX(); // Meters
            double yPose = this.getTargetDistanceY(); // IDK Maybe meters
            double CurrentRobotX = drivetrain.getState().Pose.getX();
            double CurrentRobotYaw = drivetrain.getState().Pose.getRotation().getRadians();

            double targetSetPoint = CurrentRobotX + xPose - 1; //Where the robot should go
            double targetSetPointYaw = CurrentRobotYaw + targetYaw - 0;


            double drivething = drivePID.calculate(CurrentRobotX,targetSetPoint,tmr.get());
            double turnthing = turnPID.calculate(CurrentRobotYaw,targetSetPointYaw,tmr.get());
            // Current Time Stamp Never 0!!!!!!

            System.out.println("Yaw: " + targetYaw);
            System.out.println("(X) Target Distance: " + xPose);
            System.out.println("Pose: " + drivetrain.getState().Pose);
            System.out.println("Y:" + yPose);

            System.out.println(drivething);
            System.out.println(turnthing);
            System.out.println("turn cur yaw" + CurrentRobotYaw);
            System.out.println("target yaw" + targetSetPointYaw);

            drivetrain.setControl(
                drive
                    .withVelocityX(distance2tag/5)
                    .withVelocityY(0)
                    .withRotationalRate(yPose)
            );

        } else {
            drivetrain.setControl(
                drive.withVelocityX(0) // Stop moving
                .withVelocityY(0)
                .withRotationalRate(0)
            );
            currentCam = !currentCam;
            camera = (currentCam) ? topCam : bottomCam;
            System.out.println(camera); // prints id
        }
    }
    */

    public Command APT(){
        Command ret = run(()->{
            // followAprilTag();
            return;
        });
        ret.addRequirements(drivetrain);
        return ret;
    }
    
    @Override
    public void periodic(){
        latestResults = camera.getAllUnreadResults();
    }

    public Vision(CommandSwerveDrivetrain train, String Cam) {
        this.camera = new PhotonCamera(Cam);
        drivetrain = train;
        snapTo.HeadingController = new PhoenixPIDController(1, 0, 0);
        snapTo.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
        tmr.restart();
    }

    public Vision(CommandSwerveDrivetrain drivetrain) {
        camera = (currentCam) ? topCam : bottomCam;
        this.drivetrain = drivetrain;
        snapTo.HeadingController = new PhoenixPIDController(1, 0, 0);
        snapTo.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
        tmr.restart();
    }
}