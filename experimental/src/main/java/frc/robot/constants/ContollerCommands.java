package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Elevator;

public class ContollerCommands {
    //TODO, fill with commands for the controller
    static {
        //code can be put here to help set things up
        //(This runs when the class is loaded, so we can use it to setup static constants)
        Elevator elevator = Elevator.getInst();
        CoralPivot arm = CoralPivot.getInst();
        upOneLevel = Commands.parallel(
            elevator.oneLevelUp(), arm.toOutput()
        );
        
        downOneLevel = Commands.parallel(
            elevator.oneLevelDown(), arm.toOutput()
        );
        
        toInputPosition = Commands.parallel(
            elevator.toInputLevel(), arm.toIntake()
        );
    }
    
    private static Command upOneLevel;
    private static Command downOneLevel;
    private static Command toInputPosition;
    
    public static final CommandXboxController JOYSTICK = new CommandXboxController(0);
    
    public static final Command POV_UP = upOneLevel;
    public static final Command POV_DOWN = downOneLevel;
    public static final Command POV_LEFT = CoralPivot.getInst().toIntake();
    public static final Command POV_RIGHT = toInputPosition;
    
    public static final Command LEFT_TRIGGER = 
        CoralIntake.getInst().CoralIn(JOYSTICK::getLeftTriggerAxis);
    public static final Command RIGHT_TRIGGER = 
        CoralIntake.getInst().CoralIn(JOYSTICK::getRightTriggerAxis);
    
    public static final Command LEFT_BUMPER = 
        Algae.getInst().AlgaeIn();
    public static final Command RIGHT_BUMPER = 
        Algae.getInst().AlgaeOut();
    
    public static final Command A_BUTTON = null;
    public static final Command B_BUTTON = null;
    public static final Command X_BUTTON = null;
    public static final Command Y_BUTTON = null;
    
    /* Don't have access to drivetrain */
    public static Command START_BUTTON = null;
}
