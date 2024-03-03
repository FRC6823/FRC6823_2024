package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShintakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class TimedDrive extends Command {
    private SwerveDriveSubsystem swerve;
    private double x, y, z, time;
    private Timer tim;

     public TimedDrive(SwerveDriveSubsystem swerve, double x, double y, double z, double time){
        this.swerve = swerve;
        tim = new Timer();
        this.x = x;
        this.y = y;
        this.z = z;
        this.time = time;
        addRequirements(swerve);
    }

    @Override
    public void initialize(){
        tim.reset();
        tim.start();

        swerve.driveFC(new ChassisSpeeds(x, y, z));
    }

    public boolean isFinished(){
        if (tim.hasElapsed(time)){
            swerve.stop();
        }
        return tim.hasElapsed(time);
    }
}
