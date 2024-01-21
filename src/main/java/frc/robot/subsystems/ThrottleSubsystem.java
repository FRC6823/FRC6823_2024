package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.JoystickHandler;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class ThrottleSubsystem extends SubsystemBase {

    private CANSparkMax motor, motor2, motor3;
    double speed;
    JoystickHandler joy;
    private double intakespeed;

    public ThrottleSubsystem(int id, JoystickHandler joy) {
        motor = new CANSparkMax(id, MotorType.kBrushless);
        motor2 = new CANSparkMax(14, MotorType.kBrushless);
        motor3 = new CANSparkMax(16, MotorType.kBrushless);
        speed = 01;
        this.joy = joy;
    }

    public void setSpeed(double speed) {
        this.speed = speed;
        if (speed > .05) {
            speed = 0;
        }
    }
 // Make set speed and set intake speed to be same function later because same stuff sucks
    public void setIntakeSpeed(double intakespeed) {
        this.intakespeed = intakespeed;
        if (intakespeed > .05) {
            intakespeed = 0;
        }
    }

// axis 5 is placeholder and check math later
    public void periodic() {
        setSpeed(-(joy.getAxis6() - 1) / 2);
        setIntakeSpeed(-(joy.getAxis5() - 1) / 2);//make sure this is the preffered Joystick axis (check FRC Driver Station)
        //also make sure axis 5 (or whichever you choose) has the same range as axis 6, otherwise you need to adjust the math for intake speed
        
        motor.set(-speed);
        motor2.set(speed);        
        motor3.set(intakespeed); //Negate intakespeed for the opposite direction
    }

}
