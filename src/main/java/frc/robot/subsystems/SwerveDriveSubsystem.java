package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Pigeon2Handler;
import frc.robot.util.Constants;

public class SwerveDriveSubsystem extends SubsystemBase {

    public final double W = Constants.DRIVE_TRAIN_WIDTH / 2;
    public final double L = Constants.DRIVE_TRAIN_LENGTH / 2;

    private SwerveWheelModuleSubsystem backRight;
    private SwerveWheelModuleSubsystem backLeft;
    private SwerveWheelModuleSubsystem frontRight;
    private SwerveWheelModuleSubsystem frontLeft;
    private SwerveDriveKinematics kinematics;
    private ChassisSpeeds speeds;
    private PIDController angleController; //this angle Controller is never used?
    private Pigeon2Handler pigeon;
    private LimeLightSubsystem limeLight;
    private SwerveDriveOdometry odometry;
    private boolean disableDrive;

    private SwerveDrivePoseEstimator poseEstimator;
    
    public SwerveDriveSubsystem(Pigeon2Handler pigeon, LimeLightSubsystem limeLight) {

        backLeft = new SwerveWheelModuleSubsystem(3, 2, 27, "BR", Constants.bROffset);// These are the motors and encoder
                                                                 // CAN IDs for swerve drive
        backRight = new SwerveWheelModuleSubsystem(1, 8, 26, "BL", Constants.bLOffset);
        frontRight = new SwerveWheelModuleSubsystem(5, 4, 28, "FR", Constants.fROffset);
        frontLeft = new SwerveWheelModuleSubsystem(7, 6, 25, "FL", Constants.fLOffset);// The order is angle, speed,
                                                                                   // encoder, calibrateWidget
        SendableRegistry.addChild(this, backRight);
        SendableRegistry.addChild(this, backLeft);
        SendableRegistry.addChild(this, frontRight);
        SendableRegistry.addChild(this, frontLeft);

        SendableRegistry.addLW(this, "Swerve Drive Subsystem");

        angleController = new PIDController(.3, 0, 0);
        angleController.enableContinuousInput(0, Math.PI * 2);
        angleController.setSetpoint(0);
        
        Translation2d backRightLocation = new Translation2d(-L, -W);
        Translation2d backLeftLocation = new Translation2d(-L, W);
        Translation2d frontRightLocation = new Translation2d(L, -W);
        Translation2d frontLeftLocation = new Translation2d(L, W);

        kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
        speeds = new ChassisSpeeds(0, 0, 0);
        disableDrive = false;
        this.pigeon = pigeon;
        this.limeLight = limeLight;
        odometry = new SwerveDriveOdometry
                    (kinematics, 

                    pigeon.getAngleRad(), 

                    new SwerveModulePosition[] {
                        backRight.getSwerveModulePosition(), 
                        backLeft.getSwerveModulePosition(),
                        frontRight.getSwerveModulePosition(),
                        frontLeft.getSwerveModulePosition()});

        Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, pigeon.getAngleRad(), getModulePoses(), getRobotPose(), stateStdDevs, visionStdDevs);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        // implementation from Pride of the North
        speeds = chassisSpeeds;
    }

    public void toggleDrive(){
        disableDrive = !disableDrive;
    }

    public void setSwerveModuleStates(SwerveModuleState[] states)
    {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 5);
        
        // Front left module state
        SwerveModuleState frontLeftState = states[0];
        

        // Front right module state
        SwerveModuleState frontRightState = states[1];
        

        // Back left module state
        SwerveModuleState backLeftState = states[2];

        // Back right module state
        SwerveModuleState backRightState = states[3];

        frontLeft.drive(frontLeftState.speedMetersPerSecond / 5, frontLeftState.angle.getDegrees());
        frontRight.drive(-frontRightState.speedMetersPerSecond / 5, frontRightState.angle.getDegrees());
        backLeft.drive(backLeftState.speedMetersPerSecond / 5, backLeftState.angle.getDegrees()); 
        backRight.drive(-backRightState.speedMetersPerSecond / 5, backRightState.angle.getDegrees());
    }

    // @Override
    public void periodic() {
        // Convert to module states
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        if (!disableDrive)
            setSwerveModuleStates(moduleStates);

        odometry.update(pigeon.getAngleRad(), 
                        getModulePoses());

        poseEstimator.update(pigeon.getAngleRad(), getModulePoses());

        if (limeLight.lHasValidTarget()){
            poseEstimator.addVisionMeasurement(limeLight.lGetFSPose(), limeLight.lGetTime());
        }
        if (limeLight.rHasValidTarget()){
            poseEstimator.addVisionMeasurement(limeLight.rGetFSPose(), limeLight.rGetTime());
        }

        SmartDashboard.putNumber("X Pose", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Y Pose", poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Theta Pose", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    }

    public void stop() {
        backRight.stop();
        backLeft.stop();
        frontRight.stop();
        frontLeft.stop();
    }

    public void coast(){
        backRight.coast();
        backLeft.coast();
        frontRight.coast();
        frontLeft.coast();
    }

    public void brake(){
        speeds = new ChassisSpeeds(0,0,0.0001);
        backRight.brake();
        backLeft.brake();
        frontRight.brake();
        frontLeft.brake();
        
    }

    //public Rotation2d getRobotAngle()
    //{
        //return pigeon.getAngleDeg();
    //}

    public void resetPose()
    {
        odometry.resetPosition(pigeon.getAngleRad(), 
                                getModulePoses(),
                                new Pose2d(0, 0, pigeon.getAngleRad()));

        poseEstimator.resetPosition(pigeon.getAngleRad(), getModulePoses(), new Pose2d(0, 0, pigeon.getAngleRad()));
    }

    public void setPose(double x, double y, double heading)
    {
        odometry.resetPosition(pigeon.getAngleRad(), 
                                getModulePoses(),
                                new Pose2d(x, y, new Rotation2d(heading)));

        poseEstimator.resetPosition(pigeon.getAngleRad(), getModulePoses(), new Pose2d(x, y, new Rotation2d(heading)));
    }

    public void resetSensors()
    {
        backLeft.resetSensor();
        backRight.resetSensor();
        frontLeft.resetSensor();
        frontRight.resetSensor();
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public Pose2d getRobotPose()
    {
        return odometry.getPoseMeters();
    }

    public Pose2d getEstimatorPose()
    {
        return poseEstimator.getEstimatedPosition();
    }

    public SwerveModulePosition[] getModulePoses(){
        return new SwerveModulePosition[] {
            backRight.getSwerveModulePosition(), 
            backLeft.getSwerveModulePosition(),
            frontRight.getSwerveModulePosition(),
            frontLeft.getSwerveModulePosition()};
    }
}
