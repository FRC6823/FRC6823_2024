package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import java.util.Map;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Blinkin;
import frc.robot.Constants.Const;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.AnalogTriggerOutput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShintakeSubsystem extends SubsystemBase {

    private CANSparkMax topMotor, botMotor, intakeMotor;
    private double speed;
    private double intakespeed;
    private CommandJoystick joy3;
    private AnalogInput beamValue;
    private AnalogTrigger beamBreak;
    private AnalogTriggerOutput noteReady;
    private Blinkin lights;
    private ShuffleboardTab preferences = Shuffleboard.getTab("Preferences");
    private int upperBeamRange = 
        (int) preferences.add("Upper beam range", 900)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min",0,"max",4000))
            .getEntry().getInteger(900);
    private int lowerBeamRange = 
        (int)preferences.add("Lower beam range", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min",0,"max",4000))
            .getEntry().getInteger(0);

    public ShintakeSubsystem() {
        topMotor= new CANSparkMax(14, MotorType.kBrushless);
        botMotor = new CANSparkMax(12, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(16, MotorType.kBrushless);
        joy3 = new CommandJoystick(3);
        topMotor.restoreFactoryDefaults();
        botMotor.restoreFactoryDefaults();
        topMotor.setIdleMode(IdleMode.kCoast);
        botMotor.setIdleMode(IdleMode.kCoast);
        topMotor.setInverted(true);
        botMotor.setInverted(true);
        topMotor.burnFlash();
        botMotor.burnFlash();

        beamValue = new AnalogInput(0);
        beamBreak = new AnalogTrigger(beamValue);
        beamBreak.setAveraged(true);
        beamBreak.setLimitsRaw(lowerBeamRange,upperBeamRange);
        noteReady = beamBreak.createOutput(AnalogTriggerOutput.AnalogTriggerType.kInWindow);
        SmartDashboard.putNumber("Beamvalue", beamValue.getAverageValue());
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

    public void lights(Blinkin lights) {
        this.lights = lights;
    }

    public void periodic() {
        topMotor.set(-speed);
        botMotor.set(-speed);
        intakeMotor.set(intakespeed); 

        SmartDashboard.putNumber("Beamvalue", beamValue.getAverageValue());
        
        //upperBeamRange = (int)SmartDashboard.getNumber("Upper beam range", 800);
        //lowerBeamRange = (int)SmartDashboard.getNumber("Lower beam range", 600);

        beamBreak.setLimitsRaw(lowerBeamRange, upperBeamRange);

        SmartDashboard.putBoolean("Note Ready", !noteReady.get());

        if (!noteReady.get()) {
            lights.setIntakeFull();
        } else if (noteReady.get()) {
            lights.setIntakeEmpty();
        }
    }

    public void stop() {
        speed = 0;
        intakespeed = 0;
    }

}
