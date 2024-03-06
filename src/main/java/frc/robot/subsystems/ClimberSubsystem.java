package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{
    private CANSparkMax motor20;
    private CANSparkMax motor21;
    private double climberSpeed;

    public ClimberSubsystem(){
        motor20 = new CANSparkMax(20, MotorType.kBrushed);
        motor21 = new CANSparkMax(21, MotorType.kBrushed);
        climberSpeed = 0;
    }
    public void setExtendSpeed20(double climberSpeed) {
            this.climberSpeed = climberSpeed;
            motor20.set(climberSpeed);

    }
    public void setExtendSpeed21(double climberSpeed) {
            this.climberSpeed = climberSpeed;
            motor21.set(climberSpeed);

    }

    public void setRetractSpeed20(double climberSpeed) {
        this.climberSpeed = climberSpeed;
        motor20.set(climberSpeed);

    }

    public void setRetractSpeed21(double climberSpeed) {
        this.climberSpeed = climberSpeed;
        motor21.set(climberSpeed);

    }

    public void stopClimber20() {
        motor20.stopMotor();
    }

    public void stopClimber2q() {
        motor21.stopMotor();
    }
}
