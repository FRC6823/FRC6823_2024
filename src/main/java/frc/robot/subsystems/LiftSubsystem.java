package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase{
    private CANSparkMax angleMotor;
    private SparkPIDController pidController;
    private RelativeEncoder encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr; //heavily "inspired" by Rev example code
    private boolean mode; //true is position mode (default), false is velocity mode (driver controlled)
    private double setPoint;
    private double speed;
    private boolean disabled;
    private SimpleWidget liftSpeedWidget;
    
    public LiftSubsystem () {
        angleMotor = new CANSparkMax(9, MotorType.kBrushless);
        angleMotor.restoreFactoryDefaults();
        pidController = angleMotor.getPIDController();
        encoder = angleMotor.getEncoder();
        angleMotor.setIdleMode(IdleMode.kBrake);
        SendableRegistry.addLW(this, "Lift Extension");
        this.liftSpeedWidget = Shuffleboard.getTab("Preferences").addPersistent("Lift Rate", 1)
                .withWidget(BuiltInWidgets.kNumberSlider);
        setPoint = getPosition();
        speed = 0;
        disabled = true;
        // PID coefficients
        kP = .1; //5e-5
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);
        
        mode = true;
    }

    public void setMode(boolean mode)
    {
        this.mode = mode;
    }

    public void setSetPoint(double setPoint)
    {
        this.setPoint = setPoint;
        disabled = false;
    }

    public void setSpeed(double speed)
    {
        this.speed = speed;
        disabled = false;
    }

    public double getPosition()
    {
        return encoder.getPosition();
    }

    public void disable(){
        disabled = true;
    }

    public void enable(){
        disabled = false;
    }

    public boolean isAtSetPoint(){
        return getPosition() < setPoint + 0.5 && getPosition() > setPoint - 0.5;
    }

    @Override
    public void periodic()
    {
        if(!disabled){
            if(mode) {
                //setPoint = Math.min(setPoint, Constants.EXTENSION_MIN);
                //setPoint = Math.max(setPoint , Constants.EXTENSION_MAX);
                SmartDashboard.putNumber("Lift SetPt", setPoint);
                SmartDashboard.putNumber("Lift Encoder", getPosition());
                pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
            } else {
                /*if (getPosition() >= Constants.EXTENSION_MIN){
                    speed = Math.min(speed, 0);
                }
                if (getPosition() <= Constants.EXTENSION_MAX){
                    speed = Math.max(speed, 0);
                }*/
                setPoint = getPosition();
                angleMotor.set(liftSpeedWidget.getEntry().getDouble(1) * speed);
            }
        }
        else {
            angleMotor.disable();
            setPoint = getPosition();
        }
    }
}
