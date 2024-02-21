package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Const;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.AnalogTriggerOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShintakeSubsystem extends SubsystemBase {

    private CANSparkMax topMotor, botMotor, intakeMotor;
    private double speed;
    private double intakespeed;
    private CommandJoystick joy3;
    private AnalogTrigger inputBeamBreak;
    private AnalogTriggerOutput beamSignal1;
    private AnalogInput rawBeamBreak;


    public ShintakeSubsystem() {
        topMotor= new CANSparkMax(14, MotorType.kBrushless);
        botMotor = new CANSparkMax(12, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(16, MotorType.kBrushless);
        joy3 = new CommandJoystick(3);
        inputBeamBreak = new AnalogTrigger(0);
        inputBeamBreak.setLimitsRaw(1000, 3000); //placeholder, also doesn't actually work figure this out later
        beamSignal1 = inputBeamBreak.createOutput(AnalogTriggerOutput.AnalogTriggerType.kInWindow);
        rawBeamBreak = new AnalogInput(0);
        topMotor.restoreFactoryDefaults();
        botMotor.restoreFactoryDefaults();
        topMotor.setIdleMode(IdleMode.kCoast);
        botMotor.setIdleMode(IdleMode.kCoast);
        topMotor.burnFlash();
        botMotor.burnFlash();
    }

    public void setShootSpeed(double speed) {
        MathUtil.applyDeadband(speed, Const.Shintake.sSpeedDeadband, Const.Shintake.sSpeedMax);
        /*
         * This is taking the -1 to 1 range of the joystick and converts it to 0 to 1
         */
        this.speed = (speed+1)/2;
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
        
       SmartDashboard.putNumber("BeamBreak", rawBeamBreak.getValue());

        if (beamSignal1.get()) {
                intakeMotor.set(0);   
        } else {
                intakeMotor.set(intakespeed); 
            }  
    }

}
