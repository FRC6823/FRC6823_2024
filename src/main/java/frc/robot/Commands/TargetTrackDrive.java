package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private double x, y, a, b, c;

    public TargetTrackDrive(SwerveDriveSubsystem swerve, ArmSubsystem arm, LimeLightSubsystem ll, CommandJoystick joy3){
        this.swerve = swerve;
        this.arm = arm;
        this.ll = ll;
        this.joy3 = joy3;
        yawPid = new PIDController(Const.SwerveDrive.yawKp, Const.SwerveDrive.yawKi, 0);
        yawPid.setSetpoint(0);
        x = 0;
        y = 0;
        a = -0.2218;
        b = 0.475;//0.473
        c = -0.9618;
        SmartDashboard.putNumber("a", a);
        SmartDashboard.putNumber("b", b);
        SmartDashboard.putNumber("c", c);
    }

    public void execute(){
        if (ll.fHasValidTarget()){
            x = -joy3.getRawAxis(1) * (-(joy3.getRawAxis(2) - 1.25)/4.25) * Const.SwerveDrive.MaxSpeed;
            y = -joy3.getRawAxis(0) * (-(joy3.getRawAxis(2) - 1.25)/4.25) * Const.SwerveDrive.MaxSpeed;

            yawPid.setP(Const.SwerveDrive.yawKp * ((Math.abs(x) + y * y)/21 + 1));

            swerve.driveFC(new ChassisSpeeds(x, y, yawPid.calculate(ll.fGetTx())));
            //Sets arm angle to desired angle based on distance
            arm.goToAngle(a * Math.exp(c * (ll.fGet3dTX() * ll.fGet3dTX() + ll.fGet3dTZ() * ll.fGet3dTZ())) + b);
        }
        else{
            swerve.driveFC(new ChassisSpeeds(((-joy3.getRawAxis(1) * (-(joy3.getRawAxis(2) - 1.25)/4.25) * Const.SwerveDrive.MaxSpeed))
                            ,(-joy3.getRawAxis(0) * (-(joy3.getRawAxis(2) - 1.25)/4.25) * Const.SwerveDrive.MaxSpeed)
                            ,(-joy3.getRawAxis(5) * (-(joy3.getRawAxis(2) - 1.25)/4.25) * Const.SwerveDrive.MaxAngularRate)));
        }
        
        SmartDashboard.putNumber("a", a);
        SmartDashboard.putNumber("b", b);
        SmartDashboard.putNumber("c", c);
    }
}
