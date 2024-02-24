package frc.robot.Constants;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Const {
    
    public class Arm{
        // PID coefficients
        public static final double kP = 5e-5;
        public static final double kP2 = 5e-5;
        public static final double kI2 = 1e-6; 
        public static final double kI = 1e-6;
        public static final double kD2 = 0;
        public static final double kD = 0; 
        public static final double kIz2 = 0;
        public static final double kIz = 0;
        public static final double kFF2 = 0.000156; 
        public static final double kFF = 0.000156; 
        public static final double kMaxOutput = 1; 
        public static final double kMaxOutput2 = 1;
        public static final double kMinOutput = -1;
        public static final double kMinOutput2 = -1;
        public static final double maxRPM2 = 5676;
        public static final double maxRPM = 5676;
        
        public static final double maxVel2 = 2000;
        public static final double maxVel = 2000;
        public static final double maxAcc2 = 1500;
        public static final double maxAcc = 1500;
        public static final double minVel2 = 0;
        public static final double minVel = 0;
        public static final double allowedErr2 = 0;
        public static final double allowedErr = 0;
    }

    public class SwerveDrive{
        public static final double DriveBaseRadius = 17.68;
        public static final double MaxSpeed = 6; // 6 meters per second desired top speed
        public static final double MaxAccel = 6;
        public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
        public static final double MaxAngularAccel = MaxAngularRate/2;
        public static final TrapezoidProfile.Constraints kTurnControlConstraints = new TrapezoidProfile.Constraints(MaxAngularRate, MaxAngularAccel);

        public static final double kP = 2.2941;//2.2941
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kA = 0.435;
        public static final double kV = 2.344;
        public static final double kS = 0.628;

        public static final double kPThetaController = 2.0; //1.5;
        public static final double kIThetaController = 0.0;
        public static final double kDThetaController = 0.0;

    }

    public class Shintake{
        //all variables start with i for intake or s for shooter
        public static final double sSpeedDeadband = 0.05;
        public static final double iSpeedDeadband = 0.05;
        public static final double sSpeedMax = 1;
        public static final double iSpeedMax = 1;
    }

    public class Limelight{
        
    }

    public static final double[] startingScorePose = new double[] {0.5, -90};
}
