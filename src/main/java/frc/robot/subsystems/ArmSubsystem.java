package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Const;

//heavily "inspired" by Rev example code

public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax motor5, motor6;
    private SparkPIDController pidController;
    private SparkPIDController pidController2;
    private SparkAbsoluteEncoder encoder;
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

        pidController.setP(Const.Arm.kP);
        pidController.setI(Const.Arm.kI);
        pidController.setD(Const.Arm.kD);
        pidController.setIZone(Const.Arm.kIz);
        pidController.setFF(Const.Arm.kFF);
        pidController.setOutputRange(Const.Arm.kMinOutput, Const.Arm.kMaxOutput);
        pidController2.setP(Const.Arm.kP);
        pidController2.setI(Const.Arm.kI);
        pidController2.setD(Const.Arm.kD);
        pidController2.setIZone(Const.Arm.kIz);
        pidController2.setFF(Const.Arm.kFF);
        pidController2.setOutputRange(Const.Arm.kMinOutput, Const.Arm.kMaxOutput);

        int smartMotionSlot = 0;
        int smartMotionSlot2 = 1;
        pidController.setSmartMotionMaxVelocity(Const.Arm.maxVel, smartMotionSlot);
        pidController.setSmartMotionMinOutputVelocity(Const.Arm.minVel, smartMotionSlot);
        pidController.setSmartMotionMaxAccel(Const.Arm.maxAcc, smartMotionSlot);
        pidController.setSmartMotionAllowedClosedLoopError(Const.Arm.allowedErr, smartMotionSlot);
        pidController2.setSmartMotionMaxVelocity(Const.Arm.maxVel2, smartMotionSlot2);
        pidController2.setSmartMotionMinOutputVelocity(Const.Arm.minVel2, smartMotionSlot2);
        pidController2.setSmartMotionMaxAccel(Const.Arm.maxAcc2, smartMotionSlot2);
        pidController2.setSmartMotionAllowedClosedLoopError(Const.Arm.allowedErr2, smartMotionSlot2);
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
        //motor5.set(setPoint);
        //motor6.set(setPoint2);
    }
}