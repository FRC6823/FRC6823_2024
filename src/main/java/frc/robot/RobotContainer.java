// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Commands.FCD;
import frc.robot.Commands.TimedShintake;
import frc.robot.Constants.Const;
import frc.robot.Constants.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShintakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {

  private CommandJoystick joy3;
  private CommandXboxController joy4;

  
  private ArmSubsystem armSubsystem;
  private SwerveDriveSubsystem drivetrain;
  private ShintakeSubsystem shintake;

  private SwerveRequest.FieldCentric drive;
  private SwerveRequest.SwerveDriveBrake brake;
  private SwerveRequest.PointWheelsAt point;
  private FCD fcd;
  private Telemetry logger;

  private TimedShintake controlledReverse;

  private PathHandler handler;

  public RobotContainer() {
    joy4 = new CommandXboxController(4);
    joy3 = new CommandJoystick(3);
    joy3.setXChannel(0);
    joy3.setYChannel(1);
    joy3.setTwistChannel(5);

    armSubsystem = new ArmSubsystem();
    drivetrain = TunerConstants.DriveTrain;
    shintake = new ShintakeSubsystem();

    fcd = new FCD(drivetrain, joy3);
    logger = new Telemetry(Const.SwerveDrive.MaxSpeed);

    handler = new PathHandler(drivetrain);

    controlledReverse = new TimedShintake(shintake, -0.1, 0.1, false);

    drivetrain.runOnce(() -> drivetrain.seedFieldRelative());
    
    configureBindings();
  }
  

 private void configureBindings() {
    // reset the field-centric heading on left bumper press
    joy3.button(3).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    //Arm Commands when button is pressed it is true and when it is released it is false 
    // - is rasing the arm
    joy3.button(13).onTrue(new InstantCommand(() -> armSubsystem.set(0.5))).onFalse(new InstantCommand(() -> armSubsystem.set(0)));
    joy3.button(14).onTrue(new InstantCommand(() -> armSubsystem.set(-0.5))).onFalse(new InstantCommand(() -> armSubsystem.set(0)));
    // joy3.button(7).onTrue(new InstantCommand(() -> armSubsystem.set(-joy3.getRawAxis(2)))).onFalse(new InstantCommand(() -> armSubsystem.set(0)));
    //joy3.button(7).onTrue(new InstantCommand()) -> armSubsystem.goToAngle(0, 0);
    //Shintake Commands
    joy3.button(1).onTrue(new InstantCommand(() -> shintake.setShootSpeed(joy3.getRawAxis(6)))).onFalse(new InstantCommand(() -> shintake.stopShooter()));
    joy3.button(6).onTrue(new InstantCommand(() -> shintake.setIntakeSpeed(.3))).onFalse(controlledReverse);
    joy3.povUp().onTrue(new InstantCommand(() -> shintake.setIntakeSpeed(-0.1))).onFalse(new InstantCommand(() -> shintake.setIntakeSpeed(0)));


    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void autonomousInit(){
    drivetrain.removeDefaultCommand();
    CommandScheduler.getInstance().cancelAll();
  }

  public Command getAutonomousCommand() {
      return handler.getPath();
  }

  public void teleopInit(){
      drivetrain.setDefaultCommand(fcd);
  }
}