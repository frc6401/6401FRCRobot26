// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Driving;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.SubsystemCommands;
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
    
    private final CommandXboxController Driver = new CommandXboxController(0);
    private final CommandJoystick Buttons = new CommandJoystick(1);
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
        () -> -0.75 * Driver.getLeftY(), 
        () -> -0.75 * Driver.getLeftX()
        //driving speed pt1 (for drive team)
    );
     
    
     public RobotContainer() {
        configureBindings();
        camera.camera();
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
        //    .onTrue(hanger.homingCommand())
        //    .onTrue(hanger.homingCommand2());
            
        
        //Button.button(10).whileTrue(subsystemCommands.aimAndShoot());
        // Driver.x().whileTrue(() -> shooter.setMotorSpeed(0.5));
        Driver.rightBumper().onTrue(new InstantCommand(() -> shooter.setPercentOutput(0.001)));
        Driver.rightBumper().onFalse(new InstantCommand(() -> shooter.setPercentOutput(0)));

        Driver.rightBumper().onTrue(new InstantCommand(() -> feeder.setPercentOutput(0.001)));
        Driver.rightBumper().onFalse(new InstantCommand(() -> feeder.setPercentOutput(0)));
      //Driver.leftBumper().whileTrue(new InstantCommand(() -> floor.set(Speed.FEED)));



        Driver.button(5).whileTrue(subsystemCommands.shootManually());
        Driver.button(2).whileTrue(intake.intakeCommand());
        Driver.button(1).onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));
        
        //Working Solution ------------------------------------------------------------------------------COMMENTED OUT 3/17/26
         while (Driver.getLeftTriggerAxis() > 0.5)
        {
            subsystemCommands.shootManually();
        }


       // Driver.button(5).onTrue(hanger.positionCommand(Hanger.Position.HANGING));
       // Driver.button(6).onTrue(hanger.positionCommand(Hanger.Position.HUNG));
    }

    private void configureManualDriveBindings() {
        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
            swerve, 
            () -> -0.75 * Driver.getLeftY(), 
            () -> -0.75 * Driver.getLeftX(),
            () -> -0.75 * Driver.getRightX()
            // driving speed pt2
        );
        swerve.setDefaultCommand(manualDriveCommand);
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
