package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Const;

//heavily "inspired" by Rev example code

public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax motor5, motor6;
    private SparkPIDController pidController;
    // private SparkPIDController pidController2;
    private SparkAbsoluteEncoder encoder;
    private SparkLimitSwitch fwd_LimitSwitch5, fwd_LimitSwitch6;
    private SparkLimitSwitch rev_LimitSwitch5, rev_LimitSwitch6;
    private double setPoint;
    // private double setPoint2;
    private double armSpeed;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
    
    public ArmSubsystem () {
        
        /*
         * To Do: Change all CAN IDs to come from Const.java
         */
        motor5 = new CANSparkMax(11, MotorType.kBrushless);
        motor6 = new CANSparkMax(15, MotorType.kBrushless);

        /*
         * Make sure you are configuring the Sparks in CODE not in firmware (via usb)
         * so that if you have to replace a Spark quickly,
         * you don't have to fight with any config except the CAN ID
        */
        motor5.restoreFactoryDefaults();
        motor6.restoreFactoryDefaults();
        motor5.follow(motor6, true);
        /*
         * Define the switches.
         * LimitSwitches are enabled upon creation, but we have the option to disable here
         */
        fwd_LimitSwitch5 = motor5.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        rev_LimitSwitch5 = motor5.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        fwd_LimitSwitch5.enableLimitSwitch(true);
        rev_LimitSwitch5.enableLimitSwitch(true);

        fwd_LimitSwitch6 = motor6.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        rev_LimitSwitch6 = motor6.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
        fwd_LimitSwitch6.enableLimitSwitch(true);
        rev_LimitSwitch6.enableLimitSwitch(true);
        
        /*
         * Resist arm movement when at "rest"
         */
        motor5.setIdleMode(IdleMode.kBrake);
        motor6.setIdleMode(IdleMode.kBrake);
        
        /* 
         * Motor 6 faces the opposite direction of motor 5, so invert it
         * Doing this allows us to send the same speed to both and get the right motion
         * 
         * To Do:
         *      -Rename these so they use robot left or robot right
         *      -Configure one motor to follow the other (follower mode)
         *          does this work with absolute encoders and limit switches?
        */
        //motor6.setInverted(true);

        //  Write the config to flash memory on the Spark Max so that the settings can survive a brownout/power outage.
        motor5.burnFlash();
        motor6.burnFlash();

        pidController = motor6.getPIDController();
        encoder = motor6.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        pidController.setFeedbackDevice(encoder);
        // pidController2 = motor6.getPIDController();
        armSpeed = 0;
        //setPoint2 = 0;
        
        setPoint = getEncoderPosition();


/*
 * PID Tuning Stuff
 */
// PID coefficients
kP = 6e-5;
kI = 0;
kD = 0;
kIz = 0;
kFF = 0.000015;
kMaxOutput = .2;
kMinOutput = -.2;
maxRPM = 5700;
// set PID coefficients
pidController.setP(kP);
pidController.setI(kI);
pidController.setD(kD);
pidController.setIZone(kIz);
pidController.setFF(kFF);
pidController.setOutputRange(kMinOutput, kMaxOutput);

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
        pidController.setReference(setPoint, CANSparkBase.ControlType.kPosition);
    }

    public void set(double speed){
        /*
         * This limits the total speed rate.  Should NOT do it this way!
         * To Do: May want to implement a Slew Rate Limiter for arm, put a global arm speed constant in dashboard?
         */
        armSpeed = speed * .1;
        motor6.set(armSpeed);
            //motor6.set(speed);
    }
    public void stop() {
        armSpeed = 0;
        motor5.stopMotor();
        motor6.stopMotor();
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("Arm Speed", armSpeed);
        SmartDashboard.putNumber("Arm Encoder", getEncoderPosition());
        SmartDashboard.putBoolean("Fwd 5 & 6 Limit Enabled", fwd_LimitSwitch5.isLimitSwitchEnabled() && fwd_LimitSwitch6.isLimitSwitchEnabled() );
        SmartDashboard.putBoolean("Rev 5 & 6 Limit Enabled", rev_LimitSwitch5.isLimitSwitchEnabled() && rev_LimitSwitch6.isLimitSwitchEnabled() );
        SmartDashboard.putBoolean("Forward Limit Status", fwd_LimitSwitch5.isPressed() && fwd_LimitSwitch6.isPressed());
        SmartDashboard.putBoolean("Reverse Limit Status", rev_LimitSwitch5.isPressed() && rev_LimitSwitch6.isPressed());
        
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain", 0);
        double i = SmartDashboard.getNumber("I Gain", 0);
        double d = SmartDashboard.getNumber("D Gain", 0);
        double iz = SmartDashboard.getNumber("I Zone", 0);
        double ff = SmartDashboard.getNumber("Feed Forward", 0);
        double max = SmartDashboard.getNumber("Max Output", 0);
        double min = SmartDashboard.getNumber("Min Output", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { pidController.setP(p); kP = p; }
        if((i != kI)) { pidController.setI(i); kI = i; }
        if((d != kD)) { pidController.setD(d); kD = d; }
        if((iz != kIz)) { pidController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { pidController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
        pidController.setOutputRange(min, max); 
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
        SmartDashboard.putNumber("ProcessVariable", encoder.getVelocity());

        // SmartDashboard.putNumber("Speed", speed);

        //motor5.set(setPoint);
        //motor6.set(setPoint2);
    }

   
}
