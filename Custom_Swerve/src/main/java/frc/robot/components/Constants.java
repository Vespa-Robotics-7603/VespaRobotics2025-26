package frc.robot.components;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.XboxController;

public class Constants {

    private Constants() {} 

    // common constants

    public static final double OFFSET = 0.350;
    public static final byte PORT = 0;
    public static final double STICK_DEADZONE_POSITIVE = 0.1;
    public static final double STICK_DEADZONE_NEGATIVE = -0.1;
    public static final double TRIGGER_DEADZONE = 0.15;


    // controller - DOESNT WORK, NOT CONSTANTS
    public static final XboxController JOYSTICKS = new XboxController(PORT);

    // public static final double X_AXIS_LEFT_STICK = JOYSTICKS.getRawAxis(0);
    // public static final double Y_AXIS_LEFT_STICK = JOYSTICKS.getRawAxis(1);

    // public static final double LEFT_TRIGGER = JOYSTICKS.getRawAxis(2);
    // public static final double RIGHT_TRIGGER = JOYSTICKS.getRawAxis(3);

    // public static final double X_AXIS_RIGHT_STICK = JOYSTICKS.getRawAxis(4);
    // public static final double Y_AXIS_RIGHT_STICK = JOYSTICKS.getRawAxis(5);


    // modules -> Front Left, Front Right, Rear Left, Rear Right.
    public static final SwerveModule MODULE_FL = new SwerveModule(new TalonFX(10),new TalonFX(11),new CANcoder(1),-1);
    public static final SwerveModule MODULE_FR = new SwerveModule(new TalonFX(40),new TalonFX(41),new CANcoder(4),1);
    public static final SwerveModule MODULE_RL = new SwerveModule(new TalonFX(20),new TalonFX(21),new CANcoder(2),-1);
    public static final SwerveModule MODULE_RR = new SwerveModule(new TalonFX(30),new TalonFX(31),new CANcoder(3),1);
    /**
     * <ul>
     * <li>Front Left -> 10 & 11, CANcoder ID: 1
     * <li>Front Right -> 40 & 41, CANcoder ID: 4
     * <li>Rear Left -> 20 & 21, CANcoder ID: 2
     * <li>Rear Right -> 30 & 31, CANcoder ID: 3
     * </ul>
     */
    public static final DriveTrain ROBOT = new DriveTrain(
        MODULE_FL,
        MODULE_FR,
        MODULE_RL,
        MODULE_RR 
    );

}
