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
    private NetworkTableEntry ftx;
    private NetworkTableEntry fty;
    //private NetworkTableEntry ltv, rtv;
    private NetworkTableEntry fb_t; //Botpose relative to Target
    private NetworkTableEntry fb_f;
    private NetworkTableEntry fid;

    public LimeLightSubsystem(){
        frontTable = NetworkTableInstance.getDefault().getTable("limelight-left");
        backTable = NetworkTableInstance.getDefault().getTable("limelight-right");
        ftx = frontTable.getEntry("tx");
        fty = frontTable.getEntry("ty");
        //ltv = leftTable.getEntry("tv");
        fb_t = frontTable.getEntry("botpose_targetspace");
        fid = frontTable.getEntry("tid");
        //rtv = rightTable.getEntry("tv");

        fb_f = frontTable.getEntry("botpose");
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

    //3d tracking methods
    public double[] fGetTargetSpacePose() {
        return fb_t.getDoubleArray(new double[]{0,0,0,0,0,0});
    }
    

    public boolean fHasValidTarget(){
        return fGetTargetSpacePose()[1] != 0;  
    }

    public double fGet3dTX() {
        return fGetTargetSpacePose()[0];
    }
    public double fGet3dTZ() {
        return fGetTargetSpacePose()[2];
    }

    public double fGet3dRY() {
        return fGetTargetSpacePose()[4];
    }

    public Pose2d fGetFSPose(){
        double[] poseArr = fb_f.getDoubleArray(new double[]{0,0,0,0,0,0,0});
        return new Pose2d(poseArr[0], poseArr[1], new Rotation2d(poseArr[5]));
    }


    public double fGetTime(){
        return fb_f.getDoubleArray(new double[]{0,0,0,0,0,0,0,0})[6];
    }

    public void periodic() {
        SmartDashboard.putNumber("Front TX", fGet3dTX());
        SmartDashboard.putNumber("Front TZ", fGet3dTZ());
        SmartDashboard.putNumber("Front RY", fGet3dRY());

    }
}