package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Telemetry;

/**
 * Generated by Pheonix Tuner X
 * 
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class SwerveDriveSubsystem extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private Telemetry telemetry;
    private CommandJoystick joy3;
    private Translation2d velocities;
    private double vx, vy, vTheta;


    public SwerveDriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public SwerveDriveSubsystem(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Pose2d getPose(){

        return m_odometry.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose){
        this.seedFieldRelative(pose);
    }

    public ChassisSpeeds getCurrSpeed(){
        return new ChassisSpeeds(vx, vy, vTheta);
    }

    public void driveFC(ChassisSpeeds speeds){
        this.setControl(new SwerveRequest.FieldCentric()
                            .withVelocityX(speeds.vxMetersPerSecond)
                            .withVelocityY(speeds.vyMetersPerSecond)
                            .withRotationalRate(speeds.omegaRadiansPerSecond));
        
        vx = speeds.vxMetersPerSecond;
        vy = speeds.vyMetersPerSecond;
        vTheta = speeds.omegaRadiansPerSecond;
    }

    public void driveRC(ChassisSpeeds speeds){
        this.setControl(new SwerveRequest.RobotCentric()
                            .withVelocityX(speeds.vxMetersPerSecond)
                            .withVelocityY(speeds.vyMetersPerSecond)
                            .withRotationalRate(speeds.omegaRadiansPerSecond));
        
        vx = speeds.vxMetersPerSecond;
        vy = speeds.vyMetersPerSecond;
        vTheta = speeds.omegaRadiansPerSecond;
    }

    public SwerveDriveKinematics getKinematics(){
        return m_kinematics;
    }

    public boolean getBool(){
        return false;
    }

    public void periodic(){
        SmartDashboard.putNumber("xpose", m_odometry.getEstimatedPosition().getX());
        SmartDashboard.putNumber("ypose", m_odometry.getEstimatedPosition().getY());
        SmartDashboard.putNumber("thetapose", m_odometry.getEstimatedPosition().getRotation().getRadians());
    }
}