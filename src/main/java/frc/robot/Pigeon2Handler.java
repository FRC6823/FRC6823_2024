package frc.robot;

//import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

import com.ctre.phoenix6.configs.MountPoseConfigs;
//import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
//import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix6.hardware.Pigeon2;

public class Pigeon2Handler {
    private Pigeon2 pigeon;
    private double pitchOffset = 0;
    private double rollOffset = 0;
    //private double initialAngle;

    //public double getInitialAngle() {
        //return initialAngle;
    //}

    //public void setInitialAngle() {
        //initialAngle = getAngleRad();
    //}

    public void setPitchOffset(double pitch){
        pitchOffset = pitch;
    }

    public void setRollOffset(double roll){
        rollOffset = roll;
    }

    public Pigeon2 getAhrs() {
        return pigeon;
    }

    public Pigeon2Handler() {
        try {
            /***********************************************************************
             * pigeon2-MXP: - Communication via RoboRIO MXP (SPI, I2C) and USB. - See
             * http://pigeon2-mxp.kauailabs.com/guidance/selecting-an-interface.
             ***********************************************************************/
            pigeon = new Pigeon2(18);
            Pigeon2Configurator pigeonconfigurator = pigeon.getConfigurator();
            MountPoseConfigs pigeonMountConfig = new MountPoseConfigs();
            pigeonMountConfig.MountPoseYaw = -90;
            pigeonMountConfig.MountPoseRoll = 0;
            pigeonMountConfig.MountPosePitch = 0;
            pigeonconfigurator.apply(pigeonMountConfig);

            //pigeon.configMountPose(-90,0, 0 ,100);
            printEverything();
        } catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating pigeon:  " + ex.getMessage(), true);
        }
    }

    public void printEverything() {
        // SmartDashboard.putNumber("getAngle()", ahrs.getAngle());

        // SmartDashboard.putNumber("getDisplacementX()", ahrs.getDisplacementX());
        // SmartDashboard.putNumber("getDisplacementY()", ahrs.getDisplacementY());
        // SmartDashboard.putNumber("getDisplacementZ()", ahrs.getDisplacementZ());

        // SmartDashboard.putNumber("getVelocityX()", ahrs.getVelocityX());
        // SmartDashboard.putNumber("getVelocityY()", ahrs.getVelocityY());
        // SmartDashboard.putNumber("getVelocityZ()", ahrs.getVelocityZ());

        SmartDashboard.putNumber("Pigeon Pitch", pigeon.getPitch().getValue());
        SmartDashboard.putNumber("Pigeon Roll", pigeon.getRoll().getValue());
        SmartDashboard.putNumber("Pigeon Yaw", pigeon.getYaw().getValue());
        //SmartDashboard.putNumber("pigeon2 Angle", MathUtil.mod(getAngleRad(), 2 * Math.PI));
    }

    public Rotation2d getAngleRad() {
        return Rotation2d.fromRadians(MathUtil.mod(pigeon.getYaw().getValue() * 2 * Math.PI / 360, Math.PI * 2));
    }

    public Rotation2d getAngleDeg() {
        return Rotation2d.fromDegrees(pigeon.getYaw().getValue());
    }

    public double getYaw() {
        double yaw = pigeon.getYaw().getValue() + (360.0 * 5);
        yaw %= 360;
        return yaw;
    }

    public double getYaw180() {
        double yaw = pigeon.getYaw().getValue() + (360.0 * 5);
        yaw %= 360;
        if (yaw >= 180){
            yaw -= 360;
        }
        return yaw;
    }

    public double getPitch(){
        return pigeon.getPitch().getValue() - pitchOffset;
    }

    public double getRoll() {
        return pigeon.getRoll().getValue() - rollOffset;
    }

    public void zeroYaw() {
        pigeon.setYaw(0);
        //pigeon.setAccumZAngle(0);
    }

    public void setYaw(double yaw) {
        pigeon.setYaw(yaw);
        //pigeon.setAccumZAngle(yaw);
    }

    /*public double getVelocity() {
        return Math.sqrt(Math.pow(ahrs.getVelocityX(), 2) + Math.pow(ahrs.getVelocityY(), 2));
    }*/
}
