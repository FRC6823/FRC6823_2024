package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Const;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShintakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SetPosition extends Command {
    private ArrayList<double[]> positions;
    private int index;

    private SwerveDriveSubsystem swerveDriveSubsystem;
    private ArmSubsystem armSubsystem;
    private ShintakeSubsystem shintakeSubsystem;

    public SetPosition (SwerveDriveSubsystem swerveDriveSubsystem, ShintakeSubsystem shintakeSubsystem, ArmSubsystem armSubsystem) {
        index = 0;
        positions = new ArrayList<double[]>();

        positions.add(Const.startingScorePose);

        public void usePose(int i) {
            shintakeSubsystem.setShootSpeed(positions.get(i)[1]);
            armSubsystem.goToAngle(positions.get(i)[0]);
        }
    }
}
