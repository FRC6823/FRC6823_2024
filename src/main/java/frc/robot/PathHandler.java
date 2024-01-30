/*import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveSubsystem extends SubsystemBase {
    public DriveSubsystem()

    AutoBuilder.configureHolonomic(
        this::getRobotPose, 
        this::resetPose, 
        this::getRobotRelativeSpeeds, 
        this::driveRobotRelative, 
        new HolonomicPathFollowerConfig(
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(5, 0.0, 0.0), 
            4.5, 
            0.4,
            new ReplanningConfig()
            ),
() -> {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
},
this
    );
}
public class RobotContainer {
    // ...

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Example Auto");
    }
};*/