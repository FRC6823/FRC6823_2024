package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class WaitUntilPose extends Command{
    private ArmSubsystem arm;

    public WaitUntilPose(ArmSubsystem arm){
        this.arm = arm;
        addRequirements(arm);
    }

    public boolean isFinished(){
        return arm.isAtSetPoint();
    }
}
