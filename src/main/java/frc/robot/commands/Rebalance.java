
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Pigeon2Handler;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.MathUtil;

public class Rebalance extends Command {
    private SwerveDriveSubsystem swerveDriveSubsystem;
    private Pigeon2Handler pigeon2;
    private PIDController pid, yawPid;

    public Rebalance(Pigeon2Handler pigeon2, SwerveDriveSubsystem swerveDriveSubsystem) {
        addRequirements(swerveDriveSubsystem);
        this.pigeon2 = pigeon2;
        this.swerveDriveSubsystem = swerveDriveSubsystem;
        pid = new PIDController(.1, 0, 0);
        yawPid = new PIDController(Constants.yawKp, Constants.yawKi, 0);
        yawPid.enableContinuousInput(0, 360);
    }

    public void initialize() {
        pid.setSetpoint(0);
        yawPid.setSetpoint(180);
    }

    @Override
    public void execute() {
        if (MathUtil.clipToZero(pigeon2.getRoll(), 10) != 0) { // old speed: -0.54
            swerveDriveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                    MathUtil.getSign(pid.calculate(pigeon2.getRoll())) * -0.52,
                    0,
                    yawPid.calculate(pigeon2.getYaw()),
                    pigeon2.getAngleDeg()));
        } else {
            swerveDriveSubsystem.brake();
        }
    }

    public boolean isFinished() {
        return false;
    }

}