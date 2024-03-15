package frc.robot.Commands;

import java.awt.geom.Point2D;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.Const;
import frc.robot.Constants.LinearInterpolationTable;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class TargetTrackDrive extends Command{
    private SwerveDriveSubsystem swerve;
    private ArmSubsystem arm;
    private LimeLightSubsystem ll;
    private CommandJoystick joy3;
    private PIDController yawPid, yPid;
    private double x, y;
    private LinearInterpolationTable armTable;
    private double setpoint = 0;

    public TargetTrackDrive(SwerveDriveSubsystem swerve, ArmSubsystem arm, LimeLightSubsystem ll, CommandJoystick joy3){
        armInterpolationTable();
        this.swerve = swerve;
        this.arm = arm;
        this.ll = ll;
        this.joy3 = joy3;
        yawPid = new PIDController(Const.SwerveDrive.yawKp, Const.SwerveDrive.yawKi, 0);
        yawPid.setSetpoint(0);
        //Sets point so PID trys to get to 0
        yPid = new PIDController(Const.SwerveDrive.yKp, Const.SwerveDrive.yKi, 0);
        yPid.setSetpoint(0);
        x = 0;
        y = 0;
    }
    public void armInterpolationTable() {

        armTable = new LinearInterpolationTable(
                       
                new Point2D.Double(0.0,Const.Arm.DOWN_ANGLE),
                new Point2D.Double(0.9798974895,0.3996),
                new Point2D.Double(1.58002856, 0.444),
                new Point2D.Double(1.590034012, 0.441),
                new Point2D.Double(1.590034669, 0.436),
                new Point2D.Double(1.590036003, 0.443),
                new Point2D.Double(1.590034669, 0.436),
                new Point2D.Double(2.093782083, 0.458),
                new Point2D.Double(2.093792399, 0.46),
                new Point2D.Double(2.114816023, 0.455),
                new Point2D.Double(2.393214575, 0.466),
                new Point2D.Double(2.54296674, 0.467),
                new Point2D.Double(2.558083314, 0.465),
                new Point2D.Double(2.562957981, 0.469),
                new Point2D.Double(2.582944616, 0.471),
                new Point2D.Double(2.588793974, 0.464));
        /*armTable = new LinearInterpolationTable(

            new Point2D.Double(0.9591711265, 0.3998),
            new Point2D.Double(0.9591711265, 0.3998),
            new Point2D.Double(1.570090956, 0.452),
            new Point2D.Double(2.030236686, 0.456),
            new Point2D.Double(2.525569243, 0.466),
            new Point2D.Double(2.525569243, 0.466),
            new Point2D.Double(2.931175362, 0.467),
            new Point2D.Double(2.931175362, 0.467),
            new Point2D.Double(0.0,Const.Arm.DOWN_ANGLE),
            new Point2D.Double(0.9798974895,0.3996),
            new Point2D.Double(1.58002856, 0.444),
            new Point2D.Double(1.590034012, 0.441),
            new Point2D.Double(1.590034669, 0.436),
            new Point2D.Double(1.590036003, 0.443),
            new Point2D.Double(1.590034669, 0.436),
            new Point2D.Double(2.093782083, 0.458),
            new Point2D.Double(2.093792399, 0.46),
            new Point2D.Double(2.114816023, 0.455),
            new Point2D.Double(2.393214575, 0.466),
            new Point2D.Double(2.54296674, 0.467),
            new Point2D.Double(2.558083314, 0.465),
            new Point2D.Double(2.562957981, 0.469),
            new Point2D.Double(2.582944616, 0.471),
            new Point2D.Double(2.588793974, 0.464)

        );*/
    }
    public void execute(){
        //SPEAKER lineup
        if (ll.fHasValidTarget()){
            if ((ll.fGetId() == 7) || (ll.fGetId() ==4)){
                x = -joy3.getRawAxis(1) * (-(joy3.getRawAxis(2) - 1.25)/4.25) * Const.SwerveDrive.MaxSpeed;
                y = -joy3.getRawAxis(0) * (-(joy3.getRawAxis(2) - 1.25)/4.25) * Const.SwerveDrive.MaxSpeed;

                yawPid.setP(Const.SwerveDrive.yawKp * ((Math.abs(x) + y * y)/21 + 1));

                swerve.driveFC(new ChassisSpeeds(x, y, yawPid.calculate(ll.fGetTx())));
                //Sets arm angle to desired angle based on distance
                setpoint = armTable.getOutput(Math.sqrt(Math.pow(ll.fGet3dTX(), 2) + Math.pow(ll.fGet3dTZ(), 2)));
                arm.goToAngle(setpoint);
            }
            
            //AMP lineup
            else if ((ll.fGetId() == 5) || (ll.fGetId() == 6)){
                x = -joy3.getRawAxis(1) * (-(joy3.getRawAxis(2) - 1.25)/4.25) * Const.SwerveDrive.MaxSpeed;
                y = -joy3.getRawAxis(0) * (-(joy3.getRawAxis(2) - 1.25)/4.25) * Const.SwerveDrive.MaxSpeed;
                //SetsP for PID Controllers
                yawPid.setP(Const.SwerveDrive.yawKp * ((Math.abs(x) + y * y)/21 + 1));
                yPid.setP(Const.SwerveDrive.yKp * ((Math.abs(x) + y * y)/21 + 1));
                //Drives FC while trying to get x and yaw to 0 to line up
                swerve.driveFC(new ChassisSpeeds(0, yPid.calculate(ll.fGet3dTX()), yawPid.calculate(ll.fGetTx())));
                //Sets arm angle to AMP_Shot
                arm.goToAngle(Const.Arm.AMP_SHOT);
            }
        }
        else{
            swerve.driveFC(new ChassisSpeeds(((-joy3.getRawAxis(1) * (-(joy3.getRawAxis(2) - 1.25)/4.25) * Const.SwerveDrive.MaxSpeed))
                            ,(-joy3.getRawAxis(0) * (-(joy3.getRawAxis(2) - 1.25)/4.25) * Const.SwerveDrive.MaxSpeed)
                            ,(-joy3.getRawAxis(5) * (-(joy3.getRawAxis(2) - 1.25)/4.25) * Const.SwerveDrive.MaxAngularRate)));
        }
        
    }
}
