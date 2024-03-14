package frc.robot.Constants;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Const {
    
    public class Arm{
        // Arm angles
        public static final double ZeroOffset = 0.4772015;
        public static final double UP_ANGLE = 0.58926248550415;
        public static final double DOWN_ANGLE = 0.376505047082901;

        // PID coefficients
        public static final double kP = 20;//5e-5;
        public static final double kP2 = 5e-5;
        public static final double kI2 = 1e-6; 
        public static final double kI = 0;
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
        
        //Positional Constants
        public static final double SPEAKER_SHOT = 0.399913;
        public static final double AMP_SHOT = 0.6231921; //placeholder
        public static final double SPEAKER_SHOT_FAR_BACK = 0.42993; //COMPLETE GUESS THIS WILL NOT WORK!
    }

    public class SwerveDrive{
        public static final double DriveBaseRadius = 17.68;
        public static final double MaxSpeed = 6; // 6 meters per second desired top speed
        public static final double MaxAccel = 6;
        public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
        public static final double MaxAngularAccel = MaxAngularRate/2;
        public static final TrapezoidProfile.Constraints kTurnControlConstraints = new TrapezoidProfile.Constraints(MaxAngularRate, MaxAngularAccel);

        public static final double kP = 1.2941;//2.2941
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kA = 0.435;
        public static final double kV = 2.344;
        public static final double kS = 0.628;

        public static final double kPThetaController = 1.0; //2.0;
        public static final double kIThetaController = 0.0;
        public static final double kDThetaController = 0.0;

        //Yaw Lock PID Constants
        public static final double yawKp = 0.17;
        public static final double yawKi = 0.01;


        //Driver Constants
        public static final double deadband = 0.1;
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

    public class ClimberSubsystem {
        //These are both placeholders, finding actual min and max is VERY IMPORTANT
        public static final double extentionMIN = -0.5;
        public static final double extentionMAX = -95; 
    }

}
