package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShintakeSubsystem;

public class TimedShintake extends Command{
    private ShintakeSubsystem shintake;
    private Timer timer;
    private double speed, time, counter;
    private boolean shoot;
    private boolean reverseShoot;

    public TimedShintake(ShintakeSubsystem shintake, double speed, double time, boolean shoot, boolean reverseShoot){
        this.shintake = shintake;
        timer = new Timer();
        this.speed = speed;
        this.time = time;
        this.shoot = shoot;
        this.reverseShoot  = reverseShoot;
        addRequirements(shintake);
    }

    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        
        if (shoot){
            shintake.setShootSpeed(speed);
            counter = 40;
        }
        else if (reverseShoot){
            shintake.setShootSpeed(speed * 0.2);
            counter = 40;
        }
        else{
            shintake.setIntakeSpeed(speed);
            shintake.hardStopShooter();
            counter = -1;
        }
    }

    public void execute(){
        if (counter == 0){
            shintake.setIntakeSpeed(0.3);
        }
        else {
            counter--;
        }
    }

    public boolean isFinished(){
        if (timer.hasElapsed(time)){
            shintake.stop();
        }
        return timer.hasElapsed(time);
    }

}
