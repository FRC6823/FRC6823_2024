package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLightSubsystem extends SubsystemBase{
    
    //All items for the left limelight will be prefixed with "l"
    //All items for the right limelight will be prefixed with "r"

    private NetworkTable frontTable;
    private NetworkTable backTable;
    private NetworkTableEntry ftx, btx;
    private NetworkTableEntry fty, bty;
    //private NetworkTableEntry ltv, rtv;
    private NetworkTableEntry fb_t, bb_t; //Botpose relative to Target
    private NetworkTableEntry fb_f, bb_f;
    private NetworkTableEntry fid, bid;

    public LimeLightSubsystem(){
        frontTable = NetworkTableInstance.getDefault().getTable("limelight-lenny");
        backTable = NetworkTableInstance.getDefault().getTable("limelight-right");
        ftx = frontTable.getEntry("tx");
        fty = frontTable.getEntry("ty");
        //ltv = leftTable.getEntry("tv");
        fb_t = frontTable.getEntry("targetpose_robotspace");
        fid = frontTable.getEntry("tid");

        btx = backTable.getEntry("tx");
        bty = backTable.getEntry("ty");
        //rtv = rightTable.getEntry("tv");
        bb_t = backTable.getEntry("targetpose_robotspace");
        bid = backTable.getEntry("tid");

        fb_f = frontTable.getEntry("botpose");
        bb_f = backTable.getEntry("botpose");

        SendableRegistry.addLW(this, "LimeLight");
    }

    //Pipeline management methods
    public void fSetPipeline(int pipeline){
        frontTable.getEntry("pipeline").setNumber(pipeline);
    }

    public void bSetPipeline(int pipeline){
        backTable.getEntry("pipeline").setNumber(pipeline);
    }

    public int fGetPipeline(){
        return (int) frontTable.getEntry("getpipe").getDouble(0);
    }

    public int bGetPipeline(){
        return (int) backTable.getEntry("getpipe").getDouble(0);
    }

    //General info methods
    public double fGetTx() {
        return ftx.getDouble(0);
    }
    public double fGetTy() {
        return fty.getDouble(0);
    }
    public double fGetId() {
        return fid.getDouble(0);
    }

    public double bGetTx() {
        return btx.getDouble(0);
    }
    public double bGetTy() {
        return bty.getDouble(0);
    }
    public double bGetId() {
        return bid.getDouble(0);
    }

    //3d tracking methods
    public double[] fGetTargetSpacePose() {
        return fb_t.getDoubleArray(new double[]{0,0,0,0,0,0});
    }
    
    public double[] bGetTargetSpacePose() {
        return bb_t.getDoubleArray(new double[]{0,0,0,0,0,0});
    }

    public boolean fHasValidTarget(){
        return fGetTargetSpacePose()[1] != 0;  
    }

    public boolean bHasValidTarget(){
        return bGetTargetSpacePose()[1] != 0;  
    }

    public double fGet3dTX() {
        return fGetTargetSpacePose()[0];
    }
    public double fGet3dTZ() {
        return fGetTargetSpacePose()[2];
    }

    public double bGet3dTX() {
        return bGetTargetSpacePose()[0];
    }
    public double bGet3dTZ() {
        return bGetTargetSpacePose()[2];
    }

    public double bGet3dRY() {
        return bGetTargetSpacePose()[4];
    }
    public double fGet3dRY() {
        return fGetTargetSpacePose()[4];
    }

    public Pose2d fGetFSPose(){
        double[] poseArr = fb_f.getDoubleArray(new double[]{0,0,0,0,0,0,0});
        return new Pose2d(poseArr[0], poseArr[1], new Rotation2d(poseArr[5]));
    }

    public Pose2d bGetFSPose(){
        double[] poseArr = bb_f.getDoubleArray(new double[]{0,0,0,0,0,0,0});
        return new Pose2d(poseArr[0], poseArr[1], new Rotation2d(poseArr[5]));
    }

    public double fGetTime(){
        return fb_f.getDoubleArray(new double[]{0,0,0,0,0,0,0,0})[6];
    }

    public double bGetTime(){
        return bb_f.getDoubleArray(new double[]{0,0,0,0,0,0,0,0})[6];
    }

    public void periodic() {
        SmartDashboard.putNumber("Front TX", fGet3dTX());
        SmartDashboard.putNumber("Front TZ", fGet3dTZ());
        SmartDashboard.putNumber("Front RY", fGet3dRY());

    }
}