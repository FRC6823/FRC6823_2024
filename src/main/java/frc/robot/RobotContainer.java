// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Commands.FCD;
import frc.robot.Commands.TargetTrackDrive;
import frc.robot.Commands.TimedDrive;
import frc.robot.Commands.TimedShintake;
import frc.robot.Commands.WaitUntilPose;
import frc.robot.Constants.Const;
import frc.robot.Constants.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.ShintakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RobotContainer {

  private CommandJoystick hotas3;
  private CommandXboxController gamepad4;

  
  private ArmSubsystem armSubsystem;
  private SwerveDriveSubsystem drivetrain;
  private ShintakeSubsystem shintake;
  private ClimberSubsystem climberSubsystem;

  private SwerveRequest.FieldCentric drive;
  private SwerveRequest.SwerveDriveBrake brake;
  private SwerveRequest.PointWheelsAt point;
  private FCD fcd;
  private Telemetry logger;

  private TimedShintake controlledReverse;
  private TargetTrackDrive tracking;
  private LimeLightSubsystem ll;
  //private Blinkin lights;

  private PathHandler handler;

  private SendableChooser<Integer> autoChooser;

  public RobotContainer() {
    gamepad4 = new CommandXboxController(5);
    hotas3 = new CommandJoystick(3);
    hotas3.setXChannel(0);
    hotas3.setYChannel(1);
    hotas3.setTwistChannel(5);

    armSubsystem = new ArmSubsystem();
    drivetrain = TunerConstants.DriveTrain;
    shintake = new ShintakeSubsystem();
    ll = new LimeLightSubsystem();
    //lights = new Blinkin();
    climberSubsystem = new ClimberSubsystem();

    fcd = new FCD(drivetrain, hotas3);
    logger = new Telemetry(Const.SwerveDrive.MaxSpeed);
    tracking = new TargetTrackDrive(drivetrain, armSubsystem, ll, hotas3);

    handler = new PathHandler(drivetrain);

    autoChooser = new SendableChooser<Integer>();
    autoChooser.setDefaultOption("Shoot", 0);
    autoChooser.addOption("2 Piece", 1);
    autoChooser.addOption("Testing (DO NOT USE)", 100);

    controlledReverse = new TimedShintake(shintake, -0.1, 0.1, false, false);

    drivetrain.resetFC(0);
   
    configureBindings();
  }
  

 private void configureBindings() {
    // reset the field-centric heading on left bumper press
    hotas3.button(3).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    hotas3.button(9).onTrue(new InstantCommand(() -> armSubsystem.set(0.3))).onFalse(new InstantCommand(() -> armSubsystem.stop()));
    hotas3.button(10).onTrue(new InstantCommand(() -> armSubsystem.set(-0.3))).onFalse(new InstantCommand(() -> armSubsystem.stop()));
 
    //Shintake Commands
    hotas3.button(6).onTrue(new InstantCommand(() -> shintake.setShootSpeed((hotas3.getRawAxis(6)+1)/2))).onFalse(new InstantCommand(() -> shintake.hardStopShooter()));
    hotas3.button(1).onTrue(new InstantCommand(() -> shintake.setIntakeSpeed(.3))).onFalse(controlledReverse);
    //joy3.povUp().onTrue(new InstantCommand(() -> shintake.setIntakeSpeed(-0.1))).onFalse(new InstantCommand(() -> shintake.setIntakeSpeed(0)));
    
    hotas3.button(4).onTrue(new InstantCommand(() -> armSubsystem.goToAngle(Const.Arm.UP_ANGLE)));
    hotas3.povUp().onTrue(new InstantCommand(() -> armSubsystem.goToAngle(Const.Arm.subwooferShot)));
    hotas3.povDown().onTrue(new InstantCommand(() -> armSubsystem.goToAngle(Const.Arm.DOWN_ANGLE)));
    hotas3.button(2).toggleOnTrue(tracking);
    
    //Arm controls
    gamepad4.button(5).onTrue(new InstantCommand(() -> armSubsystem.set(-gamepad4.getRawAxis(1)))).onFalse(new InstantCommand(() -> armSubsystem.stop()));
    
    gamepad4.button(6).onTrue(new InstantCommand(() -> climberSubsystem.setEnabled(gamepad4, true))).onFalse(new InstantCommand(() -> climberSubsystem.stop())); //tandem
    gamepad4.axisGreaterThan(3, 0.5).onTrue(new InstantCommand(() -> climberSubsystem.setEnabled(gamepad4, false))).onFalse(new InstantCommand(() -> climberSubsystem.stop())); //independent

    gamepad4.povUp().onTrue(new InstantCommand(() -> armSubsystem.goToAngle(Const.Arm.UP_ANGLE)));
    gamepad4.povRight().onTrue(new InstantCommand(() -> armSubsystem.goToAngle(Const.Arm.subwooferShot)));
    gamepad4.povDown().onTrue(new InstantCommand(() -> armSubsystem.goToAngle(Const.Arm.DOWN_ANGLE)));
    gamepad4.axisGreaterThan(2,0.5).onTrue(new InstantCommand(() -> shintake.setIntakeSpeed(0.3))).onFalse(controlledReverse);
    gamepad4.axisGreaterThan(2,0.5).onTrue(new InstantCommand(() -> shintake.setShootSpeed(-0.1)));
    gamepad4.button(1).onTrue(new InstantCommand(() -> shintake.setShootSpeed(0.80))).onFalse(new InstantCommand(() -> shintake.stopShooter()));
    gamepad4.button(3).onTrue(new InstantCommand(() -> shintake.setIntakeSpeed(0.3))).onFalse(new InstantCommand(() -> shintake.stopIntake()));

    //new InstantCommand(() -> lights.lightsNormal());

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
      //return handler.getPath();
      return getACG(2);
  }

  public void teleopInit(){
      drivetrain.setDefaultCommand(fcd);
      drivetrain.resetFC(0);
  }

  public Command getACG(int num){
    if (num == 1){
      return new SequentialCommandGroup(
                  //new InstantCommand(() -> drivetrain.resetFC((Math.PI) * 0.5)),
                  new InstantCommand(() -> armSubsystem.goToAngle(Const.Arm.subwooferShot)),
                  new WaitUntilPose(armSubsystem),
                  new TimedShintake(shintake, 0.6, 1.5, true, false),
                  new InstantCommand(() -> armSubsystem.goToAngle(Const.Arm.UP_ANGLE)),
                  new WaitUntilPose(armSubsystem),
                  new ParallelCommandGroup(new TimedDrive(drivetrain, -2, -0.1, 0, 3), 
                                          new TimedShintake(shintake, 0.5, 3, false, false))
                  /*new TimedShintake(shintake, 0.6, 1.5, true)*/);
    }

    if (num == 2){
      return new SequentialCommandGroup(
                  new InstantCommand(() -> armSubsystem.goToAngle(Const.Arm.subwooferShot)),
                  new WaitUntilPose(armSubsystem),
                  new TimedShintake(shintake, 0.6, 1.5, true, false));
    }
    return new TimedShintake(shintake, 0.4, 2, true, false);
  }
}