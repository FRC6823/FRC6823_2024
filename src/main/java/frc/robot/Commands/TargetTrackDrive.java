package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.Const;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class TargetTrackDrive extends Command{
    private SwerveDriveSubsystem swerve;
    private ArmSubsystem arm;
    private LimeLightSubsystem ll;
    private CommandJoystick joy3;
    private PIDController yawPid;

    public TargetTrackDrive(SwerveDriveSubsystem swerve, ArmSubsystem arm, LimeLightSubsystem ll, CommandJoystick joy3){
        this.swerve = swerve;
        this.arm = arm;
        this.ll = ll;
        this.joy3 = joy3;
        yawPid = new PIDController(Const.SwerveDrive.yawKp, Const.SwerveDrive.yawKi, 0);
    }

    public void execute(){
        swerve.driveFC(new ChassisSpeeds(((-joy3.getRawAxis(1) * (-(joy3.getRawAxis(2) - 1.25)/4.25) * Const.SwerveDrive.MaxSpeed)),
                                    (-joy3.getRawAxis(0) * (-(joy3.getRawAxis(2) - 1.25)/4.25) * Const.SwerveDrive.MaxSpeed),
                                    yawPid.calculate(ll.bGet3dRY()/*to be replaced with calculated values*/)));
        //Sets arm angle to desired angle based on distance
        arm.goToAngle(/*to be replaced with power curve*/0);
    }
}
