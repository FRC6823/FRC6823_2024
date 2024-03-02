package frc.robot.Commands;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.Const;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.Robot;

public class Raiming {
    private static final String Constants = null;
    private Robot robot;
    private LimeLightSubsystem limeLightSubsystem;
    private SwerveDriveSubsystem driveTrain;
    private PIDController xPid, yawPid, yPid, ryPid;
    private Pigeon2 pigeon;
    private String node;
    private double kMaxAngularSpeed = Const.SwerveDrive.MaxAngularRate;
    private SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
    private CommandJoystick m_Joystick;


public Raiming(SwerveDriveSubsystem driveTrain, LimeLightSubsystem limeLightsSubsystem, Pigeon2 pigeon, String node) {
  /*this.driveTrain = driveTrain;
  this.limeLightSubsystem = limeLightSubsystem;
  this.pigeon = pigeon;
  this.node = node;
  
  //left&right
  xPid = new PIDController(6, 0.001, 0.0); //put apriltag id here
  //yaw
  yawPid = new PIDController(Const.SwerveDrive.yawKP + 0.025, Const.SwerveDrive.yawKi, 0);
  //forward&back
  yPid = new PIDController(3.5, 0, 0);//put apriltag id here
  //idk
  ryPid = new PIDController(0.075, 0.00, 0);
  

  yawPid.enableContinuousInput(0, 360);
  ryPid.enableContinuousInput(0, 90);*/
} 

public void excecute() {
  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional() {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = limeLightSubsystem.fGetTx() * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= kMaxAngularSpeed;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }
}

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional()
  {    
    double kP = .1;
    double targetingForwardSpeed = limeLightSubsystem.fGetTy() * kP;
    targetingForwardSpeed *= Const.SwerveDrive.MaxSpeed;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  private void drive(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_Joystick.getRawAxis(1), 0.02))
            * Const.SwerveDrive.MaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_Joystick.getRawAxis(0), 0.02))
            * Const.SwerveDrive.MaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(m_Joystick.getRawAxis(5), 0.02))
            * Const.SwerveDrive.MaxAngularRate;3

    // while the A-button is pressed, overwrite some of the driving values with the output of our limelight methods
    if(m_Joystick.button(1) != null );
    {
        final var rot_limelight = limelight_aim_proportional();
        rot = rot_limelight;

        final var forward_limelight = limelight_range_proportional();
        xSpeed = forward_limelight;

        //while using Limelight, turn off field-relative driving.
        fieldRelative = false;
    }

    SwerveDriveSubsystem.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }
}
