package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.utility.CoralSystemPreset;

/**
 * The point of this class is to house all the variables that may need to be tuned or changed
 * often so that they are easier to find than scrolling through a massive constants class
 */
public class Settings {

    public static boolean useNormalControls = false; // false is for single player/Aidan controls
    public static boolean tuningTelemetryEnabled = true;

    public static class CoralSystemSettings {
        public static double kSE = 0.05;
        public static double kGE = 0.3;
        public static double kVE = 0.5;

        //public static double kSA = 0.001;
        //public static double kGA = 0.05; 
        //public static double kVA = 0.05; 

        public static double elevatorTolerance =.8;
        public static double armTolerance =5;
        public static double elevatorSpeedControl = 0.6;
        public static double elevatorRotationsToInches =(1.0/9.0)/*gear ratio*/ *(1.757*Math.PI/*circumference of the sprocket's pitch*/)*2;

        public static double startingPosition = 0;
        public static double maxHeight = 47;
        public static double minHeight = 0;

        public static double manualControlSpeed = 20; // max speed in inches per second 


        public static double delayBeforeStaging = 750; // milliseconds that after holding the change stage button, will cause it to skip to max/min stage

    }

    public static class CoralClawSettings {
        public static double startRoll = Math.toRadians(0);
        public static double startPitch = Math.toRadians(0);

        public static PIDController LeftDiffyPID = new PIDController(0.09, 0, 0); 
        public static PIDController RightDiffyPID = new PIDController(0.09, 0, 0); 

        // gravity power
        public static double gravityPower = 0.0;
        public static boolean useDriveCompensation = false; // helps compensate for accelerating

        //Limits
        public static double maxPitch = Math.toRadians(100.0); // radians
        public static double minPitch = Math.toRadians(-30.0);
        public static double rollRange = Math.toRadians(180.0);

        // Roller
        public static double intakePower = 0.7;
        public static double holdPower = 0.12; 
        public static double outtakePower = -0.25;

        // These are for manually telling the claw to go to a position for testing
        public static double testingPitch = 0;
        public static double testingRoll = 0;

        public static double manualPitchSpeed = 20;
        public static double manualRollSpeed = 45;

        public static double rollAngleExaggeration = 1.05; // slightly increases roll inputs so that they finish moving all the way
        public static int RollerCurrentLimit = 20;

        public static int DiffyMotorCurrentLimit = 75;
    }


    public static class ClimbingSettings{
        public static double climbingSpeed = Math.toRadians(60); // per second

        public static PIDController climbPID = new PIDController(0.6, 0, 0); 

        public static double maxEncoderPosition = Math.toRadians(115); // 0 degrees is straight up
        public static double minEncoderPosition = Math.toRadians(-18); // 90 degrees is directly inward

        public static boolean useCurrentLimit = true;
        public static int currentLimit = 75;
    }

    public static class AlgaeArmSettings {

        public static double AlgaePivotStartAngle = Math.toRadians(135); // 150 before elevator support

        // Presets
        public static double PresetPickupAngle = Math.toRadians(25);
        public static double PresetStoredAngle = Math.toRadians(90);
        public static double PresetOuttakeAngle = Math.toRadians(80);

        // Limits
        public static double MaxPivotAngle = Math.toRadians(120);
        public static double MinPivotAngle = Math.toRadians(0);

        public static PIDController AlgaePivotPID = new PIDController(0.17, 0, 0); // 0, 0, 0
        
        public static double IntakePower = 0.7;
        public static double HoldPower = 0.25;
        public static double OuttakePower = -0.8;

        public static int algaeRollerHasIntakedCurrent = 40;
        public static int algaeRollerStallCurrent = 30;

        public static double manualControlSpeed = Math.toRadians(90); // max speed in radians per second 
        
        // gravity power
        public static double gravityPower = 0.0;
        public static boolean useDriveCompensation = false; // helps compensate for accelerating

    }

    public static class AutoTargetingSettings {

        public static boolean AutoAimingEnabled = true;
        public static PIDController AutoAimPID = new PIDController(0, 0, 0);

        public static boolean AutoDrivingEnabled = false;
        public static double AutoDrivingPower = 0;
        public static double targetPercentageOfVisionBlocked = 0.2;

        public static double searchingSpeed = 0.5;

        public static double leftReefCrosshairOffset = 0;

    }

    public static class CoralSystemPresets {

        // Elevator Height (inches), Diffy Pitch (degrees), Diffy Roll (degrees)
        public static CoralSystemPreset GroundIntake = new CoralSystemPreset(0, -27, 0);
        public static CoralSystemPreset CoralStationIntake = new CoralSystemPreset(11, 75, 0);
        public static CoralSystemPreset L4Reef = new CoralSystemPreset(50, 85, 90);
        public static CoralSystemPreset L3Reef = new CoralSystemPreset(25, 55, 90);
        public static CoralSystemPreset L2Reef = new CoralSystemPreset(15, 55, 90);
        public static CoralSystemPreset L1Reef = new CoralSystemPreset(0, 80, 90);

    }
}