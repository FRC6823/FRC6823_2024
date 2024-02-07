package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax motor5, motor6;
    private SparkPIDController pidController;
    private SparkPIDController pidController2;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    public double kP2, kI2, kD2, kIz2, kFF2, kMaxOutput2, kMinOutput2, maxRPM2, maxVel2, minVel2, maxAcc2, allowedErr2; 
    //heavily "inspired" by Rev example code
    private double setPoint;
    private double setPoint2;
    
    public ArmSubsystem () {
        motor5 = new CANSparkMax(11, MotorType.kBrushless);
        motor6 = new CANSparkMax(15, MotorType.kBrushless);
        motor5.restoreFactoryDefaults();
        motor6.restoreFactoryDefaults();
        motor5.setIdleMode(IdleMode.kBrake);
        motor6.setIdleMode(IdleMode.kBrake);
        motor6.setInverted(true);
        pidController = motor5.getPIDController();
        pidController2 = motor6.getPIDController();

        // PID coefficients
        kP = 5e-5;
        kP2 = 5e-5;
        kI2 = 1e-6; 
        kI = 1e-6;
        kD2 = 0;
        kD = 0; 
        kIz2 = 0;
        kIz = 0;
        kFF2 = 0.000156; 
        kFF = 0.000156; 
        kMaxOutput = 1; 
        kMaxOutput2 = 1;
        kMinOutput = -1;
        kMinOutput2 = -1;
        maxRPM2 = 5676;
        maxRPM = 5676;
        
        maxVel2 = 2000;
        maxVel = 2000;
        //maxVelocity is not calculated but should work for right now.
        maxAcc2 = 1500;
        maxAcc = 1500;

        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);
        pidController2.setP(kP);
        pidController2.setI(kI);
        pidController2.setD(kD);
        pidController2.setIZone(kIz);
        pidController2.setFF(kFF);
        pidController2.setOutputRange(kMinOutput, kMaxOutput);

        int smartMotionSlot = 0;
        int smartMotionSlot2 = 1;
        pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
        pidController2.setSmartMotionMaxVelocity(maxVel2, smartMotionSlot2);
        pidController2.setSmartMotionMinOutputVelocity(minVel2, smartMotionSlot2);
        pidController2.setSmartMotionMaxAccel(maxAcc2, smartMotionSlot2);
        pidController2.setSmartMotionAllowedClosedLoopError(allowedErr2, smartMotionSlot2);
    }

    public void setSetPoint(double setPoint, double setPoint2)
    {
        this.setPoint = setPoint;
        this.setPoint2 = setPoint2;
    }

    public void set(double speed){
        motor5.set(speed);
    }
    public void set6(double speed){
        motor6.set(speed);
    }
    @Override
    public void periodic()
    {
        /*motor5.set(setPoint);
        motor6.set(setPoint2);*/
    }
}