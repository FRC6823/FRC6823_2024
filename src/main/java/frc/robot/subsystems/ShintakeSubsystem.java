package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Const;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShintakeSubsystem extends SubsystemBase {

    private CANSparkMax topMotor, botMotor, intakeMotor;
    private double speed;
    private double intakespeed;
    private CommandJoystick joy3;
    //private DigitalInput inputBeamBreak;
    //private DigitalInput shooterBeamBreak;
    //private AnalogTrigger inputBeamBreak;
    private AnalogInput inputBeamBreak;


    public ShintakeSubsystem() {
        topMotor= new CANSparkMax(14, MotorType.kBrushless);
        botMotor = new CANSparkMax(12, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(16, MotorType.kBrushless);
        joy3 = new CommandJoystick(3);
        //inputBeamBreak = new DigitalInput(0);
        //inputBeamBreak = new AnalogTrigger(0);
        inputBeamBreak = new AnalogInput(0);
        topMotor.restoreFactoryDefaults();
        botMotor.restoreFactoryDefaults();
        topMotor.setIdleMode(IdleMode.kCoast);
        botMotor.setIdleMode(IdleMode.kCoast);
        topMotor.burnFlash();
        botMotor.burnFlash();  
    }

    public void setShootSpeed(double speed) {
        topMotor.set((speed+1)/2);
        botMotor.set((speed +1 )/2);

    }
    public void stopShooter(){
        this.speed = 0;
    }

    public void stopIntake(){
        this.speed = 0;
        intakeMotor.set(speed);
    }
 
    public void setIntakeSpeed(double intakespeed) {
       intakeMotor.set((speed + 1) / 2); 
    }

    public void periodic(){
    }

}
