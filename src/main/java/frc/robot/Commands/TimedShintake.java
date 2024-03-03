package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShintakeSubsystem;

public class TimedShintake extends Command{
    private ShintakeSubsystem shintake;
    private Timer timer;
    private double speed, time;
    private boolean shoot;

    public TimedShintake(ShintakeSubsystem shintake, double speed, double time, boolean shoot){
        this.shintake = shintake;
        timer = new Timer();
        this.speed = speed;
        this.time = time;
        this.shoot = shoot;
        addRequirements(shintake);
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        
        if (shoot){
            shintake.setShootSpeed(speed);
        }
        else{
            shintake.setIntakeSpeed(speed);
        }
    }

    public boolean isFinished(){
        if (timer.hasElapsed(time)){
            shintake.setIntakeSpeed(0);
            shintake.stopShooter();
        }
        return timer.hasElapsed(time);
    }

}