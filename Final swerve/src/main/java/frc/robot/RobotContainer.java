// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.CageClimber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private SlewRateLimiter Xfilter = new SlewRateLimiter(5);
    private SlewRateLimiter Yfilter = new SlewRateLimiter(5);

    /* Setting up bindings for necessary control of the swerve drive platform */
    //private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    //        .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
    //        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 5% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    //private final SwerveRequest.RobotCentric driveRob = new SwerveRequest.RobotCentric()
    //        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    //        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    //SUBSYSTEMS
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator elevator = new Elevator();
    public final CoralPivot arm = new CoralPivot();
    public final CoralIntake intake = new CoralIntake();
    public final AlgaeIntake algaeIn = new AlgaeIntake();
    
    ;
    
    public CageClimber climber;

    public RobotContainer() {
        configureBindings();
        NamedCommands.registerCommand("Lift Elevator", elevator.goToLevelCommand(3));
        NamedCommands.registerCommand("Deposite Coral", intake.CoralOutCommand());
        NamedCommands.registerCommand("Elevator Intake Level", elevator.goToLevelCommand(1));
        NamedCommands.registerCommand("Pickup Coral", intake.CoralInCommand());

    }

    private void configureBindings() {
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(Xfilter.calculate(joystick.getLeftY() * MaxSpeed)) // Drive forward with negative Y (forward)
                    .withVelocityY(Yfilter.calculate(joystick.getLeftX() * MaxSpeed)) // Drive left with negative X (left)
                    .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        
        CommandScheduler.getInstance().registerSubsystem(elevator,arm, intake, algaeIn);
        
        Command upLel = Commands.parallel(elevator.oneLevelUp(), arm.toOutput());
        Command dwnLel = Commands.parallel(elevator.oneLevelDown(), arm.toOutput());
        Command inPos = Commands.parallel(elevator.toStart(), arm.toIntake());


        algaeIn.setDefaultCommand(algaeIn.AlgaeStopCommand());

        joystick.x().onTrue(intake.CoralInCommand());
        joystick.y().onTrue(intake.CoralOutCommand());

        joystick.povUp().onTrue(upLel);
        joystick.povDown().onTrue(dwnLel);
        joystick.povLeft().onTrue(inPos);
        joystick.leftTrigger().whileTrue(algaeIn.AlgaeInCommand());
        joystick.rightTrigger().whileTrue(algaeIn.AlgaeOutCommand());
        // joystick.x().or(joystick.y()).onFalse(algaeIn.AlgaeStopCommand());

        joystick.leftBumper().and(joystick.rightBumper()).onTrue(climber.climbCommand());

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));


        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // This method loads the auto when it is called, however, it is recommended
        // to first load your paths/autos when code starts, then return the
        // pre-loaded auto/path
        Optional<Alliance> al = DriverStation.getAlliance();
        if(al.isPresent()){
            if(al.get() == Alliance.Blue){
               return new PathPlannerAuto("Far Left Auto");
            }
            if(al.get() == Alliance.Red){
                //TODO create a new path for red
                return null;
            }
        }
        return new PathPlannerAuto("Far Left Auto");
    }
}
