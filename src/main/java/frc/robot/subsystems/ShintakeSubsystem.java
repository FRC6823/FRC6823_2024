package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Const;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShintakeSubsystem extends SubsystemBase {

    private CANSparkMax topMotor, botMotor, intakeMotor;
    private double speed;
    private double intakespeed;
    private CommandJoystick joy3;
    private DigitalInput inputBeamBreak;
    //private DigitalInput shooterBeamBreak;

    public ShintakeSubsystem() {
        topMotor= new CANSparkMax(14, MotorType.kBrushless);
        botMotor = new CANSparkMax(12, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(16, MotorType.kBrushless);
        joy3 = new CommandJoystick(3);
        inputBeamBreak = new DigitalInput(0);
        topMotor.restoreFactoryDefaults();
        botMotor.restoreFactoryDefaults();
        topMotor.setIdleMode(IdleMode.kCoast);
        botMotor.setIdleMode(IdleMode.kCoast);
        topMotor.burnFlash();
        botMotor.burnFlash();
    }

    public void setShootSpeed(double speed) {
        this.speed = speed;
        MathUtil.applyDeadband(speed, Const.Shintake.sSpeedDeadband, Const.Shintake.sSpeedMax);
    }
 
    public void setIntakeSpeed(double intakespeed) {
        this.intakespeed = intakespeed;
        MathUtil.applyDeadband(speed, Const.Shintake.iSpeedDeadband, Const.Shintake.iSpeedMax);
    }

    public boolean getinputBeamBreak() { //not currently used
        //return if beam is broken
        return inputBeamBreak.get();
    }

    public void periodic() {
        topMotor.set(speed);
        botMotor.set(speed);
        
        SmartDashboard.putBoolean("BeamBreak", inputBeamBreak.get());
           if (inputBeamBreak.get()) {
                intakeMotor.set(0);   
            } else {
                intakeMotor.set(intakespeed); 
            }
    }

}
