package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

//import com.ctre.phoenix.sensors.AbsoluteSensorRange;

//import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MathUtil;
import frc.robot.util.Constants;

public class SwerveWheelModuleSubsystem extends SubsystemBase {
    // private final double P = .008;
    // private final double I = .00001;
    private final double P = .009;
    private final double I = .00001;

    private TalonFX angleMotor;
    private TalonFX speedMotor;
    private PIDController pidController;
    private CANcoder angleEncoder;
    // private boolean calibrateMode;
    private double encoderOffset;
    private String motorName;
    // private SimpleWidget speedLim;
    


    public SwerveWheelModuleSubsystem(int angleMotorChannel, int speedMotorChannel, int angleEncoderChannel,
            String motorName, double offset) {
        // We're using TalonFX motors on CAN.
        this.angleMotor = new TalonFX(angleMotorChannel);
        this.speedMotor = new TalonFX(speedMotorChannel);
        this.angleEncoder = new CANcoder(angleEncoderChannel); // CANCoder Encoder
        this.speedMotor.setNeutralMode(NeutralModeValue.Coast);
        // angleMotor.configSupplyCurrentLimit(Constants.kdriveCurrentLimit);
        // speedMotor.configSupplyCurrentLimit(Constants.kdriveCurrentLimit);
        this.motorName = motorName;

        this.pidController = new PIDController(P, I, 0); // This is the PID constant,
        // we're not using any
        // Integral/Derivative control but increasing the P value will make
        // the motors more aggressive to changing to angles.

        pidController.enableContinuousInput(0, 360); // This makes the PID controller
        // understand the fact that for
        // our setup, 360 degrees is the same as 0 since the wheel loops.

        SendableRegistry.addChild(this, angleMotor);
        SendableRegistry.addChild(this, speedMotor);
        SendableRegistry.addChild(this, angleEncoder);
        SendableRegistry.addLW(this, "Swerve Wheel Module");

        encoderOffset = offset;
        resetSensor();
    }

    public void drive(double speed, double angle) {
        double currentEncoderValue = getPosition();
        int reverse = setAngle(angle, currentEncoderValue);
        setSpeed(speed * reverse);

        SmartDashboard.putNumber("Swerve CANCoder " + motorName, getPosition());
    }

    public int setAngle(double angle, double currentEncoderValue) {
        angle = MathUtil.mod(angle, 360); // ensure setpoint is on scale 0-360
        int reverse = 1;
        // angle += 90;

        if (MathUtil.getCyclicalDistance(currentEncoderValue, angle, 360) > 70) {
            reverse = -1;
            angle += 180;
            angle = MathUtil.mod(angle, 360);
        }

        double pidOut = -pidController.calculate(currentEncoderValue, angle);

        angleMotor.set(pidOut);

        return reverse;
    }

    public void setSpeed(double speed) {
        speedMotor.set(speed); // sets motor speed //22150 units/100 ms at 12.4V
    }

    // this method outputs position of the encoder to the smartDashBoard, useful for
    // calibrating the encoder offsets
    public double getPosition() {
        
        ////////////////TO DO
        //We need to check our math here.  I'm not sure if getAbsolutePosition() returns the value range we're expecting.
        return MathUtil.mod(angleEncoder.getAbsolutePosition().refresh().getValue() - encoderOffset, 360);
        //return MathUtil.mod(angleEncoder.getAbsolutePosition().getValue(), 360);
    }
    public double getPositionRad() {
        ////////////////TO DO
        //We need to check our math here.  I'm not sure if getAbsolutePosition() returns the value range we're expecting.
        return MathUtil.mod(getPosition(), 360) * Math.PI / 180;
    }

    public double getDistance() {
        if (motorName.equals("BR") || motorName.equals("FR")) {
            ////////////TO DO
            //We need to check the values coming out of .getposition compared to 2023.  Might need to change to getRotorPosition() or change our math.
            return -(speedMotor.getPosition().refresh().getValue() * Constants.WHEEL_CIRCUMFERENCE)
                    / (2048 * Constants.L2_RATIO);
        }
        ////////////TO DO
        //We need to check the values coming out of .getposition compared to 2023.  Might need to change to getRotorPosition() or change our math.
        return (speedMotor.getPosition().getValue() * Constants.WHEEL_CIRCUMFERENCE) / (2048 * Constants.L2_RATIO);
    }

    public void stop() {
        speedMotor.set(0);
        angleMotor.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Swerve CANCoder " + motorName, getPosition());
    }

    public void coast() {
        speedMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void brake() {
        speedMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void resetSensor() {
        speedMotor.setPosition(0);
    }

    public SwerveModulePosition getSwerveModulePosition() {
        return new SwerveModulePosition(getDistance(), new Rotation2d(getPositionRad()));
    }
}
