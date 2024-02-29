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

import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShintakeSubsystem extends SubsystemBase {

    private CANSparkMax topMotor, botMotor, intakeMotor;
    private double speed;
    private double intakespeed;
    private AnalogTrigger BeamBreak0;
    private AnalogTrigger BeamBreak1;
    private AnalogTrigger BeamBreak2;
    /*+private AnalogInput rawBeamBreak0;
    private AnalogInput rawBeamBreak1;
    private AnalogInput rawBeamBreak2; */
    public AnalogTriggerOutput beam0Empty;
    public AnalogTriggerOutput beam1Empty;
    public AnalogTriggerOutput beam2Empty;
    public boolean pieceReady;
   

    public ShintakeSubsystem() {
        topMotor= new CANSparkMax(14, MotorType.kBrushless);
        botMotor = new CANSparkMax(12, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(16, MotorType.kBrushless);

        BeamBreak0 = new AnalogTrigger(0);
        BeamBreak0.setAveraged(true); 
        BeamBreak0.setLimitsRaw(600, 800);
        beam0Empty = BeamBreak0.createOutput(AnalogTriggerOutput.AnalogTriggerType.kInWindow);

        BeamBreak1 = new AnalogTrigger(1);
        BeamBreak1.setAveraged(true); 
        BeamBreak1.setLimitsRaw(700, 1100);
        beam1Empty = BeamBreak1.createOutput(AnalogTriggerOutput.AnalogTriggerType.kInWindow);

        BeamBreak2 = new AnalogTrigger(2);
        BeamBreak2.setAveraged(true); 
        BeamBreak2.setLimitsRaw(700, 1100);
        beam2Empty = BeamBreak2.createOutput(AnalogTriggerOutput.AnalogTriggerType.kInWindow);

        //for smartdashboard raw values but doesn't work in tandem with AnalogTrigger
        /*rawBeamBreak0= new AnalogInput(0);
        rawBeamBreak1 = new AnalogInput(1); 
        rawBeamBreak2 = new AnalogInput(2); */ 

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

    public void periodic() {
        topMotor.set(speed);
        botMotor.set(speed);
        intakeMotor.set(intakespeed);
        
       /*SmartDashboard.putNumber("rawBeamBreak0", rawBeamBreak0.getValue());
       SmartDashboard.putNumber("rawBeamBreak1", rawBeamBreak1.getValue());
       SmartDashboard.putNumber("rawBeamBreak2", rawBeamBreak2.getValue()); */
       SmartDashboard.putBoolean("BeamSignal0", beam0Empty.get());
       SmartDashboard.putBoolean("BeamSignal1", beam1Empty.get());
       SmartDashboard.putBoolean("BeamSignal2", beam2Empty.get());
       SmartDashboard.putBoolean("pieceReady", pieceReady);
       
    }

}
