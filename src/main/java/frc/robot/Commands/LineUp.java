package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;

public class LineUp extends Command{
    
    private LimeLightSubsystem ll;
    private ArmSubsystem arm;

    public LineUp(LimeLightSubsystem ll, ArmSubsystem arm){
        this.ll = ll;
        this.arm = arm;

        addRequirements(ll, arm);
    }

    
}