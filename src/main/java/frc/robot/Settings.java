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

        public static double elevatorTolerance =4;
        public static double armTolerance =5;
        public static double elevatorSpeedControl =1;

        public static double coralClawStowedAngle = 90;
    }

    public static class CoralClawSettings {
        public static double startRoll = 0;
        public static double startPitch = 0;

        // Temporary single motor diffy control
        public static PIDController LeftDiffyPID = new PIDController(0, 0, 0); // 0, 0, 0
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

        public static double AlgaeArmLowerJointStartAngle = Math.toRadians(135.5); // 128.5 // 135.5
        public static double AlgaeArmUpperJointStartAngle = Math.toRadians(244.2); // 146.5 // 239.3

        // For motion profile
        public static double maxAcceleration = 0; // in/s^2
        public static double maxDeceleration = 0; // in/s^2
        public static double maxSpeed = 0; // in/s

        public static PIDController LowerJointPID = new PIDController(0.55, 0, 0); // 1, 0, 0
        public static PIDController UpperJointPID = new PIDController(0.5, 0, 0); // 0.8, 0, 0
        public static double voltageTolerance =0.05; //tolerance for PID to stop jitter movements and to 0 out voltage

        // Feedforward
        public static boolean useFeedforward = false; //TODO finish tuning this
        public static ArmFeedforward L1Feedforward = new ArmFeedforward(0, 0, 0);
        public static ArmFeedforward L2Feedforward = new ArmFeedforward(0, 0, 0);

        // Limits
        public static double maxAngleLowerJoint = Math.toRadians(150);
        public static double minAngleLowerJoint = Math.toRadians(70); // 90
        public static double minAngleLowerJointWhenArmOnOtherSide = Math.toRadians(90); // 115
        public static double maxAngleUpperJointFromLower = Math.toRadians(170); // 0 being straight
        public static double minAngleUpperJointFromLower = Math.toRadians(-170);
        public static double maxAngleUpperJoint = Math.toRadians(245); // 0 being straight
        public static double minAngleUpperJoint = Math.toRadians(-30);
        public static double maxDistanceInX = -42; // prevents hitting the hanging mechanism
        public static double maxDistanceOutX = 17.5; // expansion limit 19.5 is at actual limit

        public static double PositionTolerance = 5; // in inches for checking when arm has reached target position

        public static double temporaryStaticPower = 0;

        // oops I made two sets of these max speeds
        public static double lowerJointMovementSpeed = 3.5; // deg/s
        public static double upperJointMovementSpeed = 10.0; // deg/s
        public static double manualLowerJointSpeed = Math.toRadians(12); // radians per second
        public static double manualUpperJointSpeed = Math.toRadians(12); // radians per second

        public static boolean includeGravityCompensation = true;
        public static double lowerJointGravityMult = 0.05; // 1 // this is way more complicated so just a multiplier (more math is done in subsystem)
        public static double upperJointGravityPower = 0.05; // 0.0895 // (l2 mass in lbs)*(386.088 in/s^2 gravity)*(l2 center of mass)*( 1/((39.37)^2*(2.205)) conversion) * (1/5*1/5 gear ratio) / (4.69 stall torque)
        public static double lowerJointGravityPower = 0.05;

        public static boolean includeAccelerationCompensation = true;
        public static double accelerationMult = 1;

    }

    public static class AlgaeRollerSettings {
        public static double IntakePower = 0.9;
        public static double HoldPower = 0.35;
        public static double OuttakePower = -0.1;
        public static double ShootPower = -1.0;
        public static double OuttakeAutoPower = -0.25;
        
        //public static int maxSmartCurrent = 15;
        //public static int secondaryCurrentLimit = 20;
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