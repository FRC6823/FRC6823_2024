package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Const;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShintakeSubsystem extends SubsystemBase {

    private CANSparkMax topMotor, botMotor, intakeMotor;
    private SparkPIDController topPIDController;

    private double speed;
    private double intakespeed;
    private DigitalInput inputBeamBreak;

    //private DigitalInput shooterBeamBreak;

    public ShintakeSubsystem() {
        topMotor    = new CANSparkMax(14, MotorType.kBrushless);
        botMotor    = new CANSparkMax(12, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(16, MotorType.kBrushless);
        
        topMotor.restoreFactoryDefaults();
        botMotor.restoreFactoryDefaults();
        intakeMotor.restoreFactoryDefaults();
        
        botMotor.follow(topMotor, false);  // !!! Check if we need to invert this?
        
        topMotor.setIdleMode(IdleMode.kCoast);
        botMotor.setIdleMode(IdleMode.kCoast);
        intakeMotor.setIdleMode(IdleMode.kBrake);

        topMotor.burnFlash();
        botMotor.burnFlash();
        intakeMotor.burnFlash();

        inputBeamBreak = new DigitalInput(0);
        
        topPIDController = topMotor.getPIDController();
        topPIDController.setP(Const.Shintake.skP);
        topPIDController.setI(Const.Shintake.skI);
        topPIDController.setD(Const.Shintake.skD);
        topPIDController.setIZone(Const.Shintake.skIz);
        topPIDController.setFF(Const.Shintake.skFF);
        topPIDController.setOutputRange(Const.Shintake.skMinOutput, Const.Shintake.skMaxOutput);
    }

    public void setShootSpeed(double speed) {
        this.speed = speed;
        MathUtil.applyDeadband(speed, Const.Shintake.sSpeedDeadband, Const.Shintake.sSpeedMax);
    }

    public void setIntakeSpeed(double intakespeed) {
        this.intakespeed = intakespeed;
        MathUtil.applyDeadband(intakespeed, Const.Shintake.iSpeedDeadband, Const.Shintake.iSpeedMax);
    }

/* 
    public boolean getinputBeamBreak() { //not currently used
        //return if beam is broken
        return inputBeamBreak.get();
    }
*/

    public void periodic() {
        topMotor.set(speed);
        //botMotor.set(speed);
        
        SmartDashboard.putBoolean("BeamBreak", inputBeamBreak.get());
           /*
            if (inputBeamBreak.get()) {
                intakeMotor.set(0);   
            } else {
                intakeMotor.set(intakespeed); 
            }
            */
            intakeMotor.set(intakespeed); 
            
    }

}
