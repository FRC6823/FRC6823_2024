package frc.robot.Constants;

public class Const {
    
    public class CAN_IDS{
        //  Might be good to put ALL CAN IDs into one place
        
        public static final int BACK_RIGHT_ROTATE   = 1;    //Check before using!!
        public static final int BACK_LEFT_DRIVE     = 2;    //Check before using!!
        public static final int BACK_LEFT_ROTATE    = 3;    //Check before using!!
        public static final int FRONT_RIGHT_DRIVE   = 4;    //Check before using!!
        public static final int FRONT_RIGHT_ROTATE  = 5;    //Check before using!!
        public static final int FRONT_LEFT_DRIVE    = 6;    //Check before using!!
        public static final int FRONT_LEFT_ROTATE   = 7;    //Check before using!!
        public static final int BACK_RIGHT_DRIVE    = 8;    //Check before using!!

        public static final int FRONT_LEFT_CANCODER = 25;   //Check before using!!
        public static final int BACK_RIGHT_CANCODER = 26;   //Check before using!!
        public static final int BACK_LEFT_CANCODER  = 27;   //Check before using!!
        public static final int FRONT_RIGHT_CANCODER= 28;   //Check before using!!

        public static final int PIGEON              = 18;    //Check before using!!

        public static final int ARM_RIGHT           = 11;   //Check before using!!
        public static final int ARM_LEFT            = 15;   //Check before using!!
        
        public static final int INTAKE              = 16;   //Check before using!!
        
        public static final int SHOOTER_RIGHT       = 35;   //Check before using!!
        public static final int SHOOTER_LEFT        = 35;   //Check before using!!
        
        public static final int CLIMB_RIGHT         = 35;   //Check before using!!
        public static final int CLIMB_LEFT          = 35;   //Check before using!!
    
    }    
    public class Arm{
        //Spark Configs
        
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
        public static final double MaxSpeed = 6; // 6 meters per second desired top speed
        public static final double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
    }

    public class Shintake{
        //all variables start with i for intake or s for shooter
		
        /*
         * Shooter
         */
        public static final double skP = 0;
		public static final double skI = 0;
		public static final double skD = 0;
		public static final double skIz = 0;
		public static final double skFF = 0;
		public static final double skMinOutput = 0;
		public static final double skMaxOutput = 0;
        
        public static final double sSpeedDeadband = 0.05;
        public static final double sSpeedMax = 1;

        /*
         * Intake
         */
        public static final double iSpeedDeadband = 0.05;
        public static final double iSpeedMax = 1;
    }

    public class Limelight{
        
    }
}
