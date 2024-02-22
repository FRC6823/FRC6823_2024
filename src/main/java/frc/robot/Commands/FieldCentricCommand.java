package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.Const;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class FieldCentricCommand extends Command{
    
    private CommandJoystick joy3;

    private SwerveDriveSubsystem swerve;
    private SwerveRequest.FieldCentric drive;

    private double x, y, spin;

    public FieldCentricCommand(SwerveDriveSubsystem swerve, CommandJoystick joy3){
        this.joy3 = joy3;
        this.swerve = swerve;
        drive = new SwerveRequest.FieldCentric()
                .withDeadband(Const.SwerveDrive.MaxSpeed * 0.1).withRotationalDeadband(Const.SwerveDrive.MaxAngularRate * 0.1) // Add a 10% deadband
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        x = 0;
        y = 0;
        spin = 0;

        addRequirements(swerve);
    }

    public void execute(){
        x = -joy3.getRawAxis(1) * (joy3.getRawAxis(2) * 6);
        y = -joy3.getRawAxis(0) * (joy3.getRawAxis(2) * 6);
        spin = joy3.getRawAxis(5) * Const.SwerveDrive.MaxAngularRate;


        swerve.applyRequest(() -> 
                            drive.withVelocityX(x)
                                .withVelocityY(y)
                                .withRotationalRate(spin)); 
        
    }
}
