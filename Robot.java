package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.apriltag.AprilTagDetector;

public class Robot extends TimedRobot {
  private VictorSPX driveLeft1 = new VictorSPX(1);
  private VictorSPX driveRight1 = new VictorSPX(3);
  private VictorSPX driveLeft2 = new VictorSPX(2);
  private VictorSPX driveRight2 = new VictorSPX(4);
  private Joystick joystick = new Joystick(0);

  // Speed variables
  double stopSpeed = 0.0;
  double spinSpeed = 0.5; // Adjust this value for spin speed
  double fastSpeed = 1.0; // Max forward speed

  @Override
  public void robotInit() {}

  PhotonCamera camera = new PhotonCamera("CamFront");

  @Override
  public void robotPeriodic() {
    // Query the latest result from PhotonVision
    List <PhotonPipelineResult> result = camera.getAllUnreadResults();
    for(int i = 0; i<result.size(); i++){
      if ((result.get(i)).hasTargets()) {
        var target = (result.get(i)).getBestTarget();
        double yaw = target.getYaw(); // Angle of the target
        double distance = target.getBestCameraToTarget().getTranslation().getZ(); // Distance to the target
        // Use this data for navigation or other purposes
      }
    }
  }


  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    UsbCamera camera = CameraServer.startAutomaticCapture();
  }

  @Override
  public void teleopPeriodic() {
    double forward = -joystick.getRawAxis(1); // Y-axis for forward/backward
    double turn = joystick.getRawAxis(0); // X-axis for turning

    double speed_left = forward + turn;
    double speed_right = forward - turn;

    // Clamp speeds to be within the range -1 to 1
    speed_left = Math.max(-1.0, Math.min(1.0, speed_left));
    speed_right = Math.max(-1.0, Math.min(1.0, speed_right));

    // Set motor outputs
    driveLeft1.set(ControlMode.PercentOutput, speed_left);
    driveLeft2.set(ControlMode.PercentOutput, speed_left);
    driveRight1.set(ControlMode.PercentOutput, speed_right);
    driveRight2.set(ControlMode.PercentOutput, speed_right);
}
  
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}