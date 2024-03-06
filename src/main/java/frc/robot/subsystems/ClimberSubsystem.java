package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    //motor 20 is right blue 
    //motor 21 is left yellow
    private CANSparkMax motor20;
    private CANSparkMax motor21;
    private double r_blueClimberSpeed;
    private double l_yellowClimberSpeed;

    public ClimberSubsystem() {
        motor20 = new CANSparkMax(20, MotorType.kBrushed);
        motor21 = new CANSparkMax(21, MotorType.kBrushed);
        r_blueClimberSpeed = 0;
        l_yellowClimberSpeed = 0; 
        
    }

//tandem lift control
    public void setExtendSpeed(double climberSpeed) {
        this.r_blueClimberSpeed = climberSpeed;
        this.l_yellowClimberSpeed = climberSpeed;
    }

//independent lift control
    public void setExtendSpeed(double lb_ClimberSpeed, double ry_ClimberSpeed) {
            this.r_blueClimberSpeed = ry_ClimberSpeed;
            this.l_yellowClimberSpeed = lb_ClimberSpeed;
    } 

    public void stop() {
        r_blueClimberSpeed = 0;
        l_yellowClimberSpeed = 0;
        motor20.stopMotor();
        motor21.stopMotor();
    }

    @Override
    public void periodic() {
        motor20.set(r_blueClimberSpeed);
        motor21.set(l_yellowClimberSpeed);
    }
}
