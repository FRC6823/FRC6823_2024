package frc.robot;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Const;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class PathHandler {

    //path handler implementation heavily "influenced" by 2930 Sonic Squirrels
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private PIDConstants translationController;
    private PIDConstants thetaController;
    private ReplanningConfig replanningConfig;

    public PathHandler(SwerveDriveSubsystem swerveDriveSubsystem)
    {
        this.swerveDriveSubsystem = swerveDriveSubsystem;

        translationController = new PIDConstants(Const.SwerveDrive.kP, 0, 0);
        thetaController = new PIDConstants(0, Const.SwerveDrive.kIThetaController, Const.SwerveDrive.kDThetaController);

        replanningConfig = new ReplanningConfig(false, false);
    }

    public Command getPath(){
        PathPlannerPath path;
        
        path = PathPlannerPath.fromPathFile("New Path");

        return new FollowPathHolonomic(path, 
        swerveDriveSubsystem::getPose, 
        swerveDriveSubsystem::getCurrSpeed, 
        swerveDriveSubsystem::driveRC, 
        translationController,
        thetaController,
        Const.SwerveDrive.MaxSpeed,
        Const.SwerveDrive.DriveBaseRadius,
        replanningConfig, 
        swerveDriveSubsystem::getBool, 
        swerveDriveSubsystem).beforeStarting(new InstantCommand(() -> swerveDriveSubsystem.tareEverything()));
    }

    
}
