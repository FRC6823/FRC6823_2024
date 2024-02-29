package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Const;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShintakeSubsystem;

public class PositionHandler extends Command{
    private ArrayList<double[]> positions;

    private int index;
    
    private ShintakeSubsystem shintakeSubsystem;
    private ArmSubsystem armSubsystem;

    public PositionHandler(ArmSubsystem armSubsystem, ShintakeSubsystem shintakeSubsystem) {
        index = 0;
        positions = new ArrayList<double[]>();

        positions.add(Const.startingScorePose);

        this.armSubsystem = armSubsystem;
        this.shintakeSubsystem = shintakeSubsystem;

    }

    public void increaseIndex() {
        if (index <= positions.size() - 2) {
            index += 1;
        }
    }

    public void decreaseIndex() {
        if (index >= positions.size() - 2) {
            index -= 1;
        }
    }

    public void setPose(int i) {
        armSubsystem.goToAngle(positions.get(i)[0]);
        shintakeSubsystem.setShootSpeed(positions.get(i)[1]);
        shintakeSubsystem.setIntakeSpeed(positions.get(i)[2]);

    }
}
