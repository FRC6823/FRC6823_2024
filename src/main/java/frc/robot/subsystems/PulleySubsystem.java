package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
//import com.revrobotics.SparkMaxAbsoluteEncoder;
//import com.revrobotics.CANSparkMax.IdleMode;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants;

public class PulleySubsystem extends SubsystemBase{

    private final SparkAbsoluteEncoder.Type kType;

    private double setPoint;
    private CANSparkMax angleMotor;
    //private SparkMaxPIDController pidController;
    private PIDController pidController;
    private SparkAbsoluteEncoder encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr; //heavily "inspired" by Rev example code
    private boolean mode; //true is position mode (default), false is velocity mode (driver controlled)                                                                              z 
    private double speed;
    private boolean disabled;
    
    public PulleySubsystem () {
        angleMotor = new CANSparkMax(10, MotorType.kBrushless);
        angleMotor.restoreFactoryDefaults();
        kType = SparkAbsoluteEncoder.Type.kDutyCycle;
        encoder = angleMotor.getAbsoluteEncoder(kType);
        //pidController = angleMotor.getPIDController();
        //pidController.setFeedbackDevice(encoder);
        angleMotor.setIdleMode(IdleMode.kBrake);
        SendableRegistry.addLW(this, "Pulley");
        speed = 0;
        setPoint = getPosition();
        disabled = true;
        
        // PID coefficients
        kP = -300; //5e-5
        kI = 0; //1e-4
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        //maxRPM = 5700;

        //maxVel = 2000; // values in rpm
        //maxAcc = 1500;

        //pidController.setP(kP);
        //pidController.setI(kI);
        //pidController.setD(kD);
        //pidController.setIZone(kIz);
        //pidController.setFF(kFF);
        //pidController.setOutputRange(kMinOutput, kMaxOutput);

        //int smartMotionSlot = 0;
        //pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        //pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        //pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        //pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        pidController = new PIDController(-100, 0, 0);
        pidController.setTolerance(0.005);
        
        mode = true;
    }


    public void setSpeed(double speed)
    {
        this.speed = speed;
        disabled = false;
    }

    public void setSetPoint(double setPoint)
    {
        this.setPoint = setPoint;
        disabled = false;
    }

    public void setMode(boolean mode){
        this.mode = mode;
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

    public void increment(){
        setPoint += 0.005;
        disabled = false;
    }

    public void decrement(){
        setPoint -= 0.005;
        disabled = false;
    }


    public boolean isAtSetPoint(){
        // return getPosition() < setPoint + 0.02 && getPosition() > setPoint - 0.02;
        return pidController.atSetpoint();
    }

    @Override
    public void periodic()
    {
        if (!disabled){
            if(mode) {
                setPoint = Math.min(setPoint, Constants.ELEVATOR_MAX);
                setPoint = Math.max(setPoint , Constants.ELEVATOR_MIN);
                SmartDashboard.putNumber("Pulley SetPt", setPoint);
                SmartDashboard.putNumber("Pulley Encoder", encoder.getPosition());
                pidController.setSetpoint(setPoint);
                if (pidController.atSetpoint()){
                    angleMotor.disable();
                }
                else{
                    angleMotor.set(pidController.calculate(getPosition()));
                }
                //pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
            } else {
                setPoint = getPosition();
                angleMotor.set(-speed);
            }
        }
        else{
            angleMotor.disable();
            setPoint = getPosition();
        }
    }
}
