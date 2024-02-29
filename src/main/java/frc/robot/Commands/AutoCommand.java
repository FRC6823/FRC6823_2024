package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.PositionHandler;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShintakeSubsystem;

public class AutoCommand extends SequentialCommandGroup{
    private ArmSubsystem armSubsystem;
    private ShintakeSubsystem shintakeSubsystem;
    private PositionHandler positionHandler;

    public AutoCommand(RobotContainer container, int autoNum) {
        positionHandler = container.getPositionHandler();

        if (autoNum == 1) {
            addCommands( new InstantCommand(() -> positionHandler.setPose(1)), new WaitCommand(1));
        }
    }
}
