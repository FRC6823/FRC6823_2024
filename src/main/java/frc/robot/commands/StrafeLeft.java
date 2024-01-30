package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Pigeon2Handler;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.Constants;

public class StrafeLeft extends Command{
    private SwerveDriveSubsystem swerve;
    private Pigeon2Handler pigeon;
    private PIDController yawPid;
    private double speed;


    public StrafeLeft(SwerveDriveSubsystem swerve, Pigeon2Handler pigeon, double speed)
    {
        addRequirements(swerve);
        this.swerve = swerve;
        this.pigeon = pigeon;
        this.speed = speed;
        yawPid = new PIDController(Constants.yawKp, Constants.yawKi, 0);
    }

    @Override
    public void initialize(){
        yawPid.setSetpoint(180);
    }

    @Override
    public void execute(){
        swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0, speed, yawPid.calculate(pigeon.getYaw()), pigeon.getAngleDeg()));
    }
}
