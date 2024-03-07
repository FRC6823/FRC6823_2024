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
        topMotor.setInverted(true);
        botMotor.setInverted(true);
        topMotor.burnFlash();
        botMotor.burnFlash();  
    }

    public void setShootSpeed(double speed) {
        MathUtil.applyDeadband(speed, Const.Shintake.sSpeedDeadband, Const.Shintake.sSpeedMax);
        /*
         * This is taking the -1 to 1 range of the joystick and converts it to 0 to 1
         */
        this.speed = (speed);
    }
    public void stopIntake(){
        this.intakespeed = 0;
    }

    public void hardStopShooter(){
        this.speed = -0.1;
    }

    public void stopShooter(){
        this.speed = 0;
    }
 
    public void setIntakeSpeed(double intakespeed) {
        MathUtil.applyDeadband(intakespeed, Const.Shintake.iSpeedDeadband, Const.Shintake.iSpeedMax);
        this.intakespeed = intakespeed;
    }

    /*public boolean getinputBeamBreak() { //not currently used
        //return if beam is broken
        return inputBeamBreak.get();
    }*/

    public void periodic() {
        topMotor.set(speed);
        botMotor.set(speed);
        
       SmartDashboard.putNumber("BeamBreak", inputBeamBreak.getValue()); //putNumber for testing, putBoolean when analogtrigger or digital input
           //if (inputBeamBreak.getValue()) {
                //intakeMotor.set(0);   
            //} else {
                intakeMotor.set(intakespeed); 
            //}  
    }

    public void stop() {
        speed = 0;
        intakespeed = 0;
    }

}
