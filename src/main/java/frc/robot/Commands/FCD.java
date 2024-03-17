package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.Const;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class FCD extends Command{
    private SwerveDriveSubsystem swerve;
    private CommandJoystick joy3;

    public FCD(SwerveDriveSubsystem swerve, CommandJoystick joy3){
        this.swerve = swerve;
        addRequirements(swerve);
        this.joy3 = joy3;
    }

    @Override
    public void execute(){
        if ((joy3.getRawAxis(0) == 0) && (joy3.getRawAxis(1) == 0) && (joy3.getRawAxis(2) == 0)){
            swerve.stop();
        }
        else{
        swerve.driveFC(new ChassisSpeeds(((-joy3.getRawAxis(1) * (-(joy3.getRawAxis(2) - 1.25)/3) * Const.SwerveDrive.MaxSpeed))
                            ,(-joy3.getRawAxis(0) * (-(joy3.getRawAxis(2) - 1.25)/3) * Const.SwerveDrive.MaxSpeed)
                            ,(-joy3.getRawAxis(5) * (-(joy3.getRawAxis(2) - 1.25)/3) * Const.SwerveDrive.MaxAngularRate)));
        }
    }
}
