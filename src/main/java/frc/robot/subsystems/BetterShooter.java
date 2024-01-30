package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

//import edu.wpi.first.networktables.NetworkTableInstance.NetworkMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BetterShooter extends SubsystemBase{
    
    private TalonFX lMotor, rMotor;
    private double speed;

    public BetterShooter(){
        this.lMotor = new TalonFX(13);
        this.rMotor = new TalonFX(15);
        lMotor.setNeutralMode(NeutralModeValue.Coast);
        rMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void setSpeed(double speed){
        this.speed = speed;
    }

    @Override
    public void periodic(){
        lMotor.set(speed);
        rMotor.set(speed);
    }
}
