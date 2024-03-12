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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
  private Blinkin lights;

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
    lights = new Blinkin(9);
    climberSubsystem = new ClimberSubsystem();

    fcd = new FCD(drivetrain, hotas3);
    logger = new Telemetry(Const.SwerveDrive.MaxSpeed);
    tracking = new TargetTrackDrive(drivetrain, armSubsystem, ll, hotas3);

    handler = new PathHandler(drivetrain);

    autoChooser = new SendableChooser<Integer>();
    autoChooser.setDefaultOption("Shoot", 1);
    autoChooser.addOption("Shoot & Move back", 2);
    autoChooser.addOption("Testing (DO NOT USE)", 100);

    controlledReverse = new TimedShintake(shintake, -0.1, 0.1, false, false);

    Shuffleboard.getTab("Preferences").add("Autonomus", autoChooser);

    drivetrain.resetFC(0);
   
    configureBindings();
  }
  

 private void configureBindings() {
    // reset the field-centric heading on left bumper press
    hotas3.button(3).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    hotas3.button(9).onTrue(new InstantCommand(() -> armSubsystem.set(0.3))).onFalse(new InstantCommand(() -> armSubsystem.stop()));
    hotas3.button(10).onTrue(new InstantCommand(() -> armSubsystem.set(-0.3))).onFalse(new InstantCommand(() -> armSubsystem.stop()));
 
    /*
     * Shintake Commands (hotas)
     */
    hotas3.button(6).onTrue(new InstantCommand(() -> shintake.setShootSpeed((hotas3.getRawAxis(6)+1)/2))).onFalse(new InstantCommand(() -> shintake.stop()));
    //hotas3.button(1).onTrue(new InstantCommand(() -> shintake.setIntakeSpeed(.3))).onFalse(controlledReverse);
    //joy3.povUp().onTrue(new InstantCommand(() -> shintake.setIntakeSpeed(-0.1))).onFalse(new InstantCommand(() -> shintake.setIntakeSpeed(0)));
    
    /*
     * Arm Presets (Hotas)
     */
    hotas3.povLeft().onTrue(new InstantCommand(() -> armSubsystem.goToAngle(Const.Arm.UP_ANGLE)));
    hotas3.povUp().onTrue(new InstantCommand(()->armSubsystem.goToAngle(Const.Arm.ampShot)));
    hotas3.povRight().onTrue(new InstantCommand(() -> armSubsystem.goToAngle(Const.Arm.subwooferShot)));
    hotas3.povDown().onTrue(new InstantCommand(() -> armSubsystem.goToAngle(Const.Arm.DOWN_ANGLE)));
    
    /*
     * Speaker shot line-up (hotas)
     */
    hotas3.button(1).whileTrue(tracking);
    
    /*
     * Arm controls (gamepad)
     */
    gamepad4.button(5).onTrue(new InstantCommand(() -> armSubsystem.set(-gamepad4.getRawAxis(1)))).onFalse(new InstantCommand(() -> armSubsystem.stop()));
    gamepad4.povUp().onTrue(new InstantCommand(() -> armSubsystem.goToAngle(Const.Arm.UP_ANGLE)));
    gamepad4.povRight().onTrue(new InstantCommand(() -> armSubsystem.goToAngle(Const.Arm.subwooferShot)));
    gamepad4.povDown().onTrue(new InstantCommand(() -> armSubsystem.goToAngle(Const.Arm.DOWN_ANGLE)));
    
    /*
     * Climber controls (gamepad)
     */
    // tandem climber mode:
    gamepad4.button(6).onTrue(new InstantCommand(() -> climberSubsystem.setEnabled(gamepad4, true))).onFalse(new InstantCommand(() -> climberSubsystem.stop()));
    // independent climber control:
    gamepad4.axisGreaterThan(3, 0.5).onTrue(new InstantCommand(() -> climberSubsystem.setEnabled(gamepad4, false))).onFalse(new InstantCommand(() -> climberSubsystem.stop()));

    
    /*
     * Shintake controls (gamepad) 
     */
    
    /*
     * Intake while running shooter backwards (to stop notes).
     * Once done, reverse intake briefly so the note isn't touching the shooter
     */
    gamepad4.axisGreaterThan(2, 0.5).onTrue(new InstantCommand(() -> shintake.setIntakeSpeed(0.3)))
                                    .onTrue(new InstantCommand(() -> shintake.setShootSpeed(-0.1)))
                                    .onFalse(controlledReverse);

    gamepad4.button(1).onTrue(new InstantCommand(() -> shintake.setShootSpeed(0.8))).onFalse(new InstantCommand(() -> shintake.stopShooter()));
    gamepad4.button(2).onTrue(new InstantCommand(() -> shintake.setShootSpeed(1))).onFalse(new InstantCommand(() -> shintake.stop()));
    gamepad4.button(3).onTrue(new InstantCommand(() -> shintake.setIntakeSpeed(0.3))).onFalse(new InstantCommand(() -> shintake.stopIntake()));
    gamepad4.button(4).onTrue(new InstantCommand(() -> shintake.setIntakeSpeed(-0.3))).onFalse(new InstantCommand(() -> shintake.stop()));

    /*
     * This seems like both the wrong place to put this light code and also the wrong way to run that method
     * Most likely, we either want to pre-define a few commands, or we should have a LEDSubsytem to which other subsystems can send requests
     *    both of which would then be triggered upon events
     */ 
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
      //return getACG(2);
      return getACG(autoChooser.getSelected());
  }

  public void teleopInit(){
      drivetrain.setDefaultCommand(fcd);
      //drivetrain.resetFC(0);
  }

  public Command getACG(int num){
    
    if (num == 1){
      /*
       * Shoot
       */
      return new SequentialCommandGroup(
                  new InstantCommand(() -> armSubsystem.goToAngle(Const.Arm.subwooferShot)),
                  new WaitUntilPose(armSubsystem),
                  new TimedShintake(shintake, 0.6, 1.5, true, false));
    }
    else if (num == 2){
      /*
       * Shoot & move back
       */
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
    else{
      /*
       * Shoot at the ground!?!?
       */
      return new TimedShintake(shintake, 0.4, 2, true, false);
    }
  }
}