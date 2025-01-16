package frc.robot.components;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.XboxController;

public class Constants {

    private Constants() {} 

    // common variables
    public static final double OFFSET = 0.350;
    public static final byte PORT = 0;

    // controller 
    public static XboxController JOYSTICKS = new XboxController(PORT);
    public static final double HORIZONTAL_MOVEMENT = -JOYSTICKS.getRawAxis(0);
    public static final double VERTICAL_MOVEMENT = -JOYSTICKS.getRawAxis(1);

    // modules -> Front Left, Front Right, Rear Left, Rear Right.
    public static final SwerveModule moduleFL = new SwerveModule(new TalonFX(10),new TalonFX(11),new CANcoder(1),OFFSET,-1);
    public static final SwerveModule moduleFR = new SwerveModule(new TalonFX(40),new TalonFX(41),new CANcoder(4),OFFSET,1);
    public static final SwerveModule moduleRL = new SwerveModule(new TalonFX(20),new TalonFX(21),new CANcoder(2),OFFSET,-1);
    public static final SwerveModule moduleRR = new SwerveModule(new TalonFX(30),new TalonFX(31),new CANcoder(3),OFFSET,1);

}
