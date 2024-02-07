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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.Const;
import frc.robot.Constants.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {

  private CommandJoystick joy3;
  private CommandXboxController joy0;

  
  private ArmSubsystem armSubsystem;
  private SwerveDriveSubsystem drivetrain;

  private SwerveRequest.FieldCentric drive;
  private SwerveRequest.SwerveDriveBrake brake;
  private SwerveRequest.PointWheelsAt point;
  private Telemetry logger;


  public RobotContainer() {
    joy0 = new CommandXboxController(0);
    joy3 = new CommandJoystick(3);

    armSubsystem = new ArmSubsystem();
    drivetrain = TunerConstants.DriveTrain;

    drive = new SwerveRequest.FieldCentric()
      .withDeadband(Const.SwerveDrive.MaxSpeed * 0.1).withRotationalDeadband(Const.SwerveDrive.MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    brake = new SwerveRequest.SwerveDriveBrake();
    point = new SwerveRequest.PointWheelsAt();
    logger = new Telemetry(Const.SwerveDrive.MaxSpeed);

    configureBindings();
  }
  

 private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX((-joy3.getRawAxis(1)) * Const.SwerveDrive.MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joy3.getRawAxis(0) * Const.SwerveDrive.MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joy3.getRawAxis(5) * Const.SwerveDrive.MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    // reset the field-centric heading on left bumper press
    joy3.button(3).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    joy3.button(8).whileTrue(drivetrain.applyRequest(() -> brake));
    joy3.button(9).whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joy3.getRawAxis(1), -joy3.getRawAxis(0)))));//Y is 0 X is 1

    //Arm Commands
    joy3.button(2).onTrue(new InstantCommand(() -> armSubsystem.set(-joy3.getRawAxis(2)))).onFalse(new InstantCommand(() -> armSubsystem.set(0)));
    joy3.button(10).onTrue(new InstantCommand(() -> armSubsystem.set6(-joy3.getRawAxis(2)))).onFalse(new InstantCommand(() -> armSubsystem.set6(0)));


    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
