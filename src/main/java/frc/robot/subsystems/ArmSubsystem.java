package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Const;

//heavily "inspired" by Rev example code

public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax motor11, motor15;
    private PIDController pidController;
    private SparkAbsoluteEncoder encoder;
    private SparkLimitSwitch fwd_LimitSwitch11, fwd_LimitSwitch15;
    private SparkLimitSwitch rev_LimitSwitch11, rev_LimitSwitch15;
    private double setPoint;
    // private double setPoint2;
    private double armSpeed;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
    private double upangle = Const.Arm.UP_ANGLE;
    private double downangle = Const.Arm.DOWN_ANGLE;
    private boolean pidEnabled;
    
    public ArmSubsystem () {

        /*
         * To Do: Change all CAN IDs to come from Const.java
         */
        motor11 = new CANSparkMax(11, MotorType.kBrushless);
        motor15 = new CANSparkMax(15, MotorType.kBrushless);

        /*
         * Make sure you are configuring the Sparks in CODE not in firmware (ie, via usb)
         * With code configuration, if you have to replace a Spark quickly,
         * you don't have to fight with any config except the CAN ID
        */
        motor11.restoreFactoryDefaults();
        motor15.restoreFactoryDefaults();
        
        /* 
         * Motor15 faces the "reverse" direction, so invert it then set motor 5 to follow it
         * Doing this allows us to send the speed to just one and get the right motion from both
         */
        motor15.setInverted(true);
        motor11.follow(motor15, true);
        
        /*
         * LimitSwitches are enabled upon creation, but we have the option to disable here
         */
        fwd_LimitSwitch11 = motor11.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        rev_LimitSwitch11 = motor11.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        fwd_LimitSwitch11.enableLimitSwitch(true);
        rev_LimitSwitch11.enableLimitSwitch(true);

        fwd_LimitSwitch15 = motor15.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        rev_LimitSwitch15 = motor15.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        fwd_LimitSwitch15.enableLimitSwitch(true);
        rev_LimitSwitch15.enableLimitSwitch(true);
        
        upangle = Const.Arm.UP_ANGLE;
        downangle = Const.Arm.DOWN_ANGLE;
        
        SmartDashboard.putNumber("down angle", downangle);
        SmartDashboard.putNumber("up angle", upangle);
		//Doesn't quite work the way I expected.  Need to troubleshoot
		motor15.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float)downangle);
		motor15.setSoftLimit(CANSparkBase.SoftLimitDirection.kForward, (float)upangle);
		motor15.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, false);
		motor15.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, false); 
		

        /*
         * Resist arm movement when at "rest"
         */
        motor11.setIdleMode(IdleMode.kBrake);
        motor15.setIdleMode(IdleMode.kBrake);
        

        //  Write the config to flash memory on the Spark Max so that the settings can survive a brownout/power outage.
        motor11.burnFlash();
        motor15.burnFlash();

        // pidController = motor15.getPIDController();
        encoder = motor15.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        encoder.setInverted(false);
        armSpeed = 0;
        
        setPoint = getEncoderPosition();
        
        
        
        /*
        * PID Tuning Stuff
        */
        // PID coefficients
        // kP = .1;
        kP = 5;
        kI = 0;
        kD = 0;
        kIz = 0;
        // kFF = 0.1000015;
        kFF = 0;
        kMaxOutput = .8;
        kMinOutput = -.8;
        maxRPM = 5700;
        
        pidController = new PIDController(kP, kI, kD);
        pidController.setTolerance(.002);
        pidEnabled = false;
        // pidController.setFeedbackDevice(encoder);
        
        // set PID coefficients
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        // pidController.setFF(kFF);
        // pidController.setOutputRange(kMinOutput, kMaxOutput);

// display PID coefficients on SmartDashboard
SmartDashboard.putNumber("P Gain", kP);
SmartDashboard.putNumber("I Gain", kI);
SmartDashboard.putNumber("D Gain", kD);
SmartDashboard.putNumber("I Zone", kIz);
SmartDashboard.putNumber("Feed Forward", kFF);
SmartDashboard.putNumber("Max Output", kMaxOutput);
SmartDashboard.putNumber("Min Output", kMinOutput);


/*
 *      pidController.setP(Const.Arm.kP);
        pidController.setI(Const.Arm.kI);
        pidController.setD(Const.Arm.kD);
        pidController.setIZone(Const.Arm.kIz);
        pidController.setFF(Const.Arm.kFF);
        pidController.setOutputRange(Const.Arm.kMinOutput, Const.Arm.kMaxOutput);
        // pidController2.setP(Const.Arm.kP);
        // pidController2.setI(Const.Arm.kI);
        // pidController2.setD(Const.Arm.kD);
        // pidController2.setIZone(Const.Arm.kIz);
        // pidController2.setFF(Const.Arm.kFF);
        // pidController2.setOutputRange(Const.Arm.kMinOutput, Const.Arm.kMaxOutput);

        int smartMotionSlot = 0;
        // int smartMotionSlot2 = 1;
        pidController.setSmartMotionMaxVelocity(Const.Arm.maxVel, smartMotionSlot);
        pidController.setSmartMotionMinOutputVelocity(Const.Arm.minVel, smartMotionSlot);
        pidController.setSmartMotionMaxAccel(Const.Arm.maxAcc, smartMotionSlot);
        pidController.setSmartMotionAllowedClosedLoopError(Const.Arm.allowedErr, smartMotionSlot);
        // pidController2.setSmartMotionMaxVelocity(Const.Arm.maxVel2, smartMotionSlot2);
        // pidController2.setSmartMotionMinOutputVelocity(Const.Arm.minVel2, smartMotionSlot2);
        // pidController2.setSmartMotionMaxAccel(Const.Arm.maxAcc2, smartMotionSlot2);
        // pidController2.setSmartMotionAllowedClosedLoopError(Const.Arm.allowedErr2, smartMotionSlot2);
 */
   
    }

    public void goToAngle(double setPoint){
        this.setPoint = setPoint;
        pidEnabled = true;
    }

    public boolean isAtSetPoint(){
        return pidController.atSetpoint();
    }

    public void set(double speed){
        /*
         * This limits the total speed rate.  Should NOT do it this way!
         * To Do: May want to implement a Slew Rate Limiter for arm, put a global arm speed constant in dashboard?
         */
        armSpeed = speed * .4;
        // motor15.set(armSpeed);
    }
    
    public void stop() {
        armSpeed = 0;
        pidEnabled = false;
        setPoint = getEncoderPosition();
        motor11.stopMotor();
        motor15.stopMotor();
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }


    @Override
    public void periodic(){
        /*
         * If PID is enabled, sets the motor output to a value calculated by the PIDController, but clamped to min and max output.
         * 
         * If PID is disabled, just use raw motor speed.
         * 
         */
        if (pidEnabled) {
            motor15.set( MathUtil.clamp(pidController.calculate(encoder.getPosition(), setPoint),
                                    kMinOutput,
                                    kMaxOutput)
                        );
        }
        else {

            motor15.set(armSpeed);
        }
        SmartDashboard.putNumber("Arm Speed", armSpeed);
        SmartDashboard.putNumber("Arm Encoder", getEncoderPosition());
        SmartDashboard.putBoolean("Fwd 5 & 6 Limit Enabled", fwd_LimitSwitch11.isLimitSwitchEnabled() && fwd_LimitSwitch15.isLimitSwitchEnabled() );
        SmartDashboard.putBoolean("Rev 5 & 6 Limit Enabled", rev_LimitSwitch11.isLimitSwitchEnabled() && rev_LimitSwitch15.isLimitSwitchEnabled() );
        SmartDashboard.putBoolean("Forward Limit Status", fwd_LimitSwitch11.isPressed() && fwd_LimitSwitch15.isPressed());
        SmartDashboard.putBoolean("Reverse Limit Status", rev_LimitSwitch11.isPressed() && rev_LimitSwitch15.isPressed());
        
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        // double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);
        double up = SmartDashboard.getNumber("up angle", 0);
        double dwn = SmartDashboard.getNumber("down angle", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { pidController.setP(p); kP = p; }
        if((i != kI)) { pidController.setI(i); kI = i; }
        if((d != kD)) { pidController.setD(d); kD = d; }
        if((iz != kIz)) { pidController.setIZone(iz); kIz = iz; }
        // if((ff != kFF)) { pidController.setFF(ff); kFF = ff; }
        if((up != upangle)) { motor15.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float)up); upangle = up; }
        if((dwn != downangle)) { motor15.setSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, (float)dwn); downangle = dwn; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
        // pidController.setOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
        }
        
        /**
         * PIDController objects are commanded to a set point using the 
         * SetReference() method.
         * 
         * The first parameter is the value of the set point, whose units vary
         * depending on the control type set in the second parameter.
         * 
         * The second parameter is the control type can be set to one of four 
         * parameters:
         *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
         *  com.revrobotics.CANSparkMax.ControlType.kPosition
         *  com.revrobotics.CANSparkMax.ControlType.kVelocity
         *  com.revrobotics.CANSparkMax.ControlType.kVoltage
         */
        // double setPoint = m_stick.getY()*maxRPM;
        // pidController.setReference(setPoint, CANSparkBase.ControlType.kPosition);
        
        SmartDashboard.putNumber("SetPoint", setPoint);
        // SmartDashboard.putNumber("ProcessVariable", encoder.getPosition());

        // SmartDashboard.putNumber("Speed", speed);

    }

   
}
