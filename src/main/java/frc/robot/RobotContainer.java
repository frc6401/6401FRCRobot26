// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Driving;
import frc.robot.commands.PrepareShotCommand;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Floor.Speed;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.util.SwerveTelemetry;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Floor floor = new Floor();
    private final Feeder feeder = new Feeder();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Hanger hanger = new Hanger();
    private final Limelight limelight = new Limelight("limelight");
    private final Camera camera = new Camera();


    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(Driving.kMaxSpeed.in(MetersPerSecond));
    
    private final CommandXboxController Driver = new CommandXboxController(2);
    private final CommandJoystick Driver1 = new CommandJoystick(0);
    private final CommandJoystick Buttons = new CommandJoystick(1);
    private final CommandJoystick DriverRotate = new CommandJoystick(2);
    
    // ------------------------------------------------------------------------------COMMENTED OUT 3/17/26
     /*private final AutoRoutines autoRoutines = new AutoRoutines(
        swerve,
        intake,
      floor,
        feeder,
        shooter,
        hood,
        hanger,
        limelight
    ); */

    private final SubsystemCommands subsystemCommands = new SubsystemCommands(
        swerve,
        intake,
        floor,
        feeder,
        shooter,
        hood,
        hanger,
        () -> -0.80 * Driver1.getX(), 
        () -> -0.80 * Driver1.getY()
        
    );
     
    
     public RobotContainer() {
        configureBindings();
        Camera camera = new Camera();
       // autoRoutines.configure();
        swerve.registerTelemetry(swerveTelemetry::telemeterize);
    }
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
      */
    private void configureBindings() {
        configureManualDriveBindings();
        limelight.setDefaultCommand(updateVisionCommand());  

        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
         .onTrue(intake.homingCommand());
        

        //SHOOTERS
        //Button.button(10).whileTrue(subsystemCommands.aimAndShoot());
        Buttons.button(1).onTrue(new InstantCommand(() -> shooter.setPercentOutput(0.65)));
        Buttons.button(1).onFalse(new InstantCommand(() -> shooter.setPercentOutput(0)));
        Buttons.button(1).onFalse(new InstantCommand(() -> shooter.setPercentOutput(0)));
        //Buttons.button(1).whileTrue(subsystemCommands.shootManually()); For future use

        //FLOOR AND FEEDERS
        Buttons.button(12).onTrue(new InstantCommand(() -> feeder.setPercentOutput(0.35)));
        Buttons.button(12).onFalse(new InstantCommand(() -> feeder.setPercentOutput(0)));
        Buttons.button(11).onTrue(new InstantCommand(() -> floor.set(Speed.FEED)));
        Buttons.button(11).onFalse(new InstantCommand(() -> floor.set(Speed.STOP)));
      
        //INTAKE
        Buttons.button(9).whileTrue(intake.intakeCommand());
        Buttons.button(10).onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));
        
        //DEBUGGING TOOLS

        //DEBUGGING ANGLE
        Driver1.button(5).onTrue(intake.runOnce(() -> intake.set(Intake.Position.TEST)));
        Driver1.button(6).onTrue(intake.runOnce(() -> intake.set(Intake.Position.TEST1)));
        Driver1.button(7).onTrue(intake.runOnce(() -> intake.set(Intake.Position.TEST2)));
        Driver1.button(8).onTrue(intake.runOnce(() -> intake.set(Intake.Position.TEST3)));
        Driver1.button(9).onTrue(intake.runOnce(() -> intake.set(Intake.Position.TEST4)));
        Driver1.button(10).onTrue(intake.runOnce(() -> intake.set(Intake.Position.TEST5)));
        Driver1.button(11).onTrue(intake.runOnce(() -> intake.set(Intake.Position.TEST6)));
        Driver1.button(12).onTrue(intake.runOnce(() -> intake.set(Intake.Position.TEST7)));
        Driver1.button(16).onTrue(new InstantCommand(() -> intake.setPivotPercentOutput(0.1)));
        Driver1.button(16).onFalse(new InstantCommand(() -> intake.setPivotPercentOutput(0)));
        
        //DEBUGGING RPM
        /*
        shooter.setDefaultCommand(shooter.run(() -> 
        {
        double slider;
        slider = (Buttons.getThrottle() + 1.0) / 2;
        shooter.setPercentOutput(slider);
        }));
        */

        



    }

    private void configureManualDriveBindings() {
        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
            swerve, 
            () -> -0.80 * Driver1.getY(), 
            () -> -0.80 * Driver1.getX(),
            () -> -0.80 * DriverRotate.getX()

        );
        swerve.setDefaultCommand(manualDriveCommand);

        //HEADINGS (NEED TO BE ASSINGED A CONTROLLER)
        Driver.povDown().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.k180deg)));
        Driver.povRight().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCW_90deg)));
        Driver.povLeft().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCCW_90deg)));
        Driver.povUp().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kZero)));
        Driver.button(8).onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.fromDegrees(45))));
       
        Driver.button(7).onTrue(Commands.runOnce(() -> manualDriveCommand.seedFieldCentric()));   
    }


    private Command updateVisionCommand() {
        return limelight.run(() -> {
            final Pose2d currentRobotPose = swerve.getState().Pose;
            final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                swerve.addVisionMeasurement(
                    m.poseEstimate.pose, 
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                );
            });
        })
        .ignoringDisable(true);
    }
    
}
