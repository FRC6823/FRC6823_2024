package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ClimberSubsystem extends SubsystemBase{
    //motor 20 is right blue 
    //motor 21 is left yellow
    private CANSparkMax motor20;
    private CANSparkMax motor21;
    private CommandXboxController gamepad4;
    private boolean isEnabled;
    private boolean tandemMode;

    public ClimberSubsystem() {
        motor20 = new CANSparkMax(20, MotorType.kBrushed);
        motor21 = new CANSparkMax(21, MotorType.kBrushed);
        tandemMode = true;

        motor20.setIdleMode(IdleMode.kBrake);
        motor21.setIdleMode(IdleMode.kBrake);
        
    }

//tandem lift control
    /*public void setExtendSpeed(double climberSpeed) {
        this.r_blueClimberSpeed = climberSpeed;
        this.l_yellowClimberSpeed = climberSpeed;
    }

//independent lift control
    public void setExtendSpeed(double lb_ClimberSpeed, double ry_ClimberSpeed) {
            this.r_blueClimberSpeed = ry_ClimberSpeed;
            this.l_yellowClimberSpeed = lb_ClimberSpeed;
    } */

    public void setEnabled(CommandXboxController gamepad4, boolean tandemMode){
        isEnabled = true;
        this.gamepad4 = gamepad4;
        this.tandemMode = tandemMode;
    }

    public void stop() {
        motor20.stopMotor();
        motor21.stopMotor();
        isEnabled = false;
    }

    @Override
    public void periodic() {
        if ( isEnabled == true){
            if (tandemMode == false){
                motor20.set((-gamepad4.getRawAxis(1)) * 0.5);
                motor21.set((gamepad4.getRawAxis(5)) * 0.5);
            }
            if (tandemMode == true) {
                motor20.set((-gamepad4.getRawAxis(5)) * 0.5);
                motor21.set((gamepad4.getRawAxis(5)) * 0.5);
            }
        }
        else{
            stop();
        }
    }
}
