package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;

/**
 * The point of this class is to house all the variables that may need to be tuned or changed
 * often so that they are easier to find than scrolling through a massive constants class
 */
public class Settings {

    public static boolean tuningTelemetryEnabled = true;

    public static class CoralSystemSettings {
        public static double kSE = 0.0001;
        public static double kGE = 0.00;
        public static double kVE = 0.1;

        public static double kSA = 0.001; // 0.001
        public static double kGA = 0.05; // 0.05
        public static double kVA = 0.05; // 0.05

        public static double elevatorTolerance =.8;
        public static double armTolerance =5;
        public static double elevatorSpeedControl =1;
         public static double elevatorRotationsToInches =(1.0/20.0)/*gear ratio*/ *(1.757*Math.PI/*circumfrance of the sprocket*/)*2;

        public static double coralClawStowedAngle = 90;
    }

    public static class CoralClawSettings {
        public static double startRoll = 0;
        public static double startPitch = 0;

        // Temporary single motor diffy control
        public static PIDController LeftDiffyPID = new PIDController(0, 0, 0); // 0, 0, 0
        public static PIDController RightDiffyPID = new PIDController(0, 0, 0); // 0, 0, 0
        public static double startRotatePosition = 0;
        public static double secondRotatePosition = 90;

        //Diffy Motor PID
        public static double kP = 0.0;
        public static double kI = 0.0;
        public static double kD = 0.0;

        // feedforward values which one motor needs to be set really high becauswe Cole and Micah suck at building and designing
        public static double RFF = 0.0;
        public static double LFF = 0.0;

        //Limits - Degrees
        public static double maxPitch = 90.0; // degrees
        public static double minPitch = -90.0;
        public static double rollRange = 180.0;

        public static double intakePower = 0.2;
        public static double holdPower = 0.05; 
        public static double outtakePower = -0.5;
        //public static int intakeSmartStallCurrent = 4;
        //public static int intakeSecondaryCurrent = 5;

        //public static int diffyMotorSmartStallCurrent = 25;

        public static double testingPitch = 0;
        public static double testingRoll = 0;

    }
    public static class ClimbingSettings{
        public static double climbingSpeed = 1;

        public static double maxEncoderPosition = 85;
        public static double minEncoderPosition = 0;
    }

    public static class AlgaeArmSettings {

        public static double AlgaePivotStartAngle = Math.toRadians(90); 

        // Presets
        public static double PresetPickupAngle = 45;
        public static double PresetStoredAngle = 90;
        public static double PresetOuttakeAngle = 90;

        // Limits
        public static double MaxPivotAngle = Math.toRadians(135);
        public static double MinPivotAngle = Math.toRadians(0);

        public static PIDController AlgaePivotPID = new PIDController(0, 0, 0); // 0, 0, 0
        
        public static double IntakePower = 0.3;
        public static double HoldPower = 0.2;
        public static double OuttakePower = 0.5;

        public static int algaeRollerHasIntakedCurrent = 4;
        public static int algaeRollerStallCurrent = 5;
        
    }

    public static class AutoTargetingSettings {

        public static boolean AutoAimingEnabled = true;
        public static PIDController AutoAimPID = new PIDController(0, 0, 0);

        public static boolean AutoDrivingEnabled = false;
        public static double AutoDrivingPower = 0;
        public static double targetPercentageOfVisionBlocked = 0.2;

        public static double searchingSpeed = 0.5;

    }
}