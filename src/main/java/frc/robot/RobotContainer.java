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
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;

public class RobotContainer {

  private ArmSubsystem armSubsystem = new ArmSubsystem();

  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandJoystick HOTAS = new CommandJoystick(3);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  /*private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));*/

 private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX((-HOTAS.getRawAxis(1)) * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-HOTAS.getRawAxis(0) * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-HOTAS.getRawAxis(5) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    HOTAS.button(8).whileTrue(drivetrain.applyRequest(() -> brake));
    HOTAS.button(9).whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-HOTAS.getRawAxis(1), -HOTAS.getRawAxis(0)))));//Y is 0 X is 1

    // reset the field-centric heading on left bumper press
    HOTAS.button(3).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    HOTAS.button(2).onTrue(new InstantCommand(() -> armSubsystem.set(-HOTAS.getRawAxis(2)))).onFalse(new InstantCommand(() -> armSubsystem.set(0)));
    HOTAS.button(10).onTrue(new InstantCommand(() -> armSubsystem.set6(-HOTAS.getRawAxis(2)))).onFalse(new InstantCommand(() -> armSubsystem.set6(0)));
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
