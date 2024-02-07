package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Const;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShintakeSubsystem extends SubsystemBase {

    private CANSparkMax topMotor, botMotor, intakeMotor;
    private double speed;
    private double intakespeed;

    public ShintakeSubsystem(int id) {
        topMotor= new CANSparkMax(14, MotorType.kBrushless);
        botMotor = new CANSparkMax(12, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(16, MotorType.kBrushless);
    }

    public void setShootSpeed(double speed) {
        this.speed = speed;
        MathUtil.applyDeadband(speed, Const.Shintake.sSpeedDeadband, Const.Shintake.sSpeedMax);
    }
 
    public void setIntakeSpeed(double intakespeed) {
        this.intakespeed = intakespeed;
        MathUtil.applyDeadband(speed, Const.Shintake.iSpeedDeadband, Const.Shintake.iSpeedMax);
    }

    public void periodic() {
        topMotor.set(speed);
        botMotor.set(speed);

        intakeMotor.set(intakespeed);
    }

}
