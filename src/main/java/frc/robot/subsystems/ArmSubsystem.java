package frc.robot.subsystems;

import java.util.concurrent.CancellationException;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Const;

//heavily "inspired" by Rev example code

public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax motor5, motor6;
    private SparkPIDController pidController;
    private SparkPIDController pidController2;
    private SparkAbsoluteEncoder encoder;
    private SparkLimitSwitch fwd_LimitSwitch5, fwd_LimitSwitch6;
    private SparkLimitSwitch rev_LimitSwitch5, rev_LimitSwitch6;
    private double setPoint;
    private double setPoint2;
    
    public ArmSubsystem () {
        motor5 = new CANSparkMax(11, MotorType.kBrushless);
        motor6 = new CANSparkMax(15, MotorType.kBrushless);

        /* Make sure you are configuring the Sparks in CODE not in the firmaware */
        motor5.restoreFactoryDefaults();
        motor6.restoreFactoryDefaults();

        fwd_LimitSwitch5 = motor5.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        rev_LimitSwitch5 = motor5.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        fwd_LimitSwitch5.enableLimitSwitch(false);
        rev_LimitSwitch5.enableLimitSwitch(true);

        fwd_LimitSwitch6 = motor6.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        rev_LimitSwitch6 = motor6.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        fwd_LimitSwitch6.enableLimitSwitch(false);
        rev_LimitSwitch6.enableLimitSwitch(true);
        
        motor5.setIdleMode(IdleMode.kBrake);
        motor6.setIdleMode(IdleMode.kBrake);
        motor6.setInverted(true);
        pidController = motor5.getPIDController();
        pidController2 = motor6.getPIDController();

        encoder = motor6.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);



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

    public void goToAngle(double setPoint, double setPoint2)
    {
        this.setPoint = setPoint;
        motor5.set(setPoint);
        motor6.set(setPoint2);
    }

    public void set(double speed){
            motor5.set(speed);
            motor6.set(speed);
    }
    public double getEncoderPosition() {
        return encoder.getPosition();
    }
    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Arm Encoder", getEncoderPosition());
        // SmartDashboard.putNumber("Speed", ((speed + 1) /2));

        //motor5.set(setPoint);
        //motor6.set(setPoint2);
    }
}
