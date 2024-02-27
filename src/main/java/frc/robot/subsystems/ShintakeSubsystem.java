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
    private double intakePiece;
    private AnalogTrigger inputBeamBreak0;
    private AnalogTrigger inputBeamBreak1;
    private AnalogTrigger inputBeamBreak2;
    private AnalogInput rawBeamBreak0;
    private AnalogInput rawBeamBreak1;
    private AnalogInput rawBeamBreak2;
    private AnalogTriggerOutput beam0Empty;
    private AnalogTriggerOutput beam1Empty;
    private AnalogTriggerOutput beam2Empty;
    private boolean pieceReady;
   

    public ShintakeSubsystem() {
        topMotor= new CANSparkMax(14, MotorType.kBrushless);
        botMotor = new CANSparkMax(12, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(16, MotorType.kBrushless);

        /*inputBeamBreak0 = new AnalogTrigger(0);
        inputBeamBreak0.setLimitsRaw(600, 800);
        beam0Empty = inputBeamBreak0.createOutput(AnalogTriggerOutput.AnalogTriggerType.kInWindow);

        inputBeamBreak1 = new AnalogTrigger(1);
        inputBeamBreak1.setLimitsRaw(600, 800); //placeholder
        beam1Empty = inputBeamBreak1.createOutput(AnalogTriggerOutput.AnalogTriggerType.kInWindow);

        inputBeamBreak2 = new AnalogTrigger(2);
        inputBeamBreak2.setLimitsRaw(600, 800); //placeholder
        beam2Empty = inputBeamBreak2.createOutput(AnalogTriggerOutput.AnalogTriggerType.kInWindow);*/

        //for smartdashboard but doesn't work in tandem with AnalogTrigger
        rawBeamBreak0 = new AnalogInput(0); 
        rawBeamBreak1 = new AnalogInput(1); 
        rawBeamBreak2 = new AnalogInput(2);


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

    /*public void intakePiece(double intakespeed) { 
        if (beam0Empty.get() && beam1Empty.get() && beam2Empty.get()) { //everything is empty the world is sad. FILL ME WITH NOTES OF JOY!
            setIntakeSpeed(intakespeed);
            pieceReady = false;
        } else if (!beam0Empty.get() && !beam1Empty.get() && beam2Empty.get()) { //a note exists but does not touch shooter (THIS IS VERY GOOD)
            setIntakeSpeed(0);
            pieceReady = true;
        } else if (!beam2Empty.get()){ //a note exists but goes too far and touches the shooter (THIS IS VERY BAD)
            do{
                setIntakeSpeed(-intakespeed);
                pieceReady = false;
            } while (beam0Empty.get());
                pieceReady = true;
        } /*else { //when things break terribly or the univerise disintigrates

        }

    } */

    public void periodic() {
        topMotor.set(speed);
        botMotor.set(speed);
        intakeMotor.set(intakespeed);
        
       SmartDashboard.putNumber("rawBeamBreak0", rawBeamBreak0.getValue());
       SmartDashboard.putNumber("rawBeamBreak1", rawBeamBreak1.getValue());
       SmartDashboard.putNumber("rawBeamBreak2", rawBeamBreak2.getValue());
       /*SmartDashboard.putBoolean("BeamSignal0", beam0Empty.get());
       SmartDashboard.putBoolean("BeamSignal1", beam1Empty.get());
       SmartDashboard.putBoolean("BeamSignal2", beam2Empty.get());
       SmartDashboard.putBoolean("pieceReady", pieceReady);*/
       
    }

}
