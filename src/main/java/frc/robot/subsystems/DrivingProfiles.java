package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;
import frc.robot.Settings.AutoTargetingSettings;
import frc.robot.subsystems.CommandSwerveDrivetrain.gyroData;
import frc.robot.utility.Functions;
import frc.robot.utility.LimelightHelpers;
import frc.robot.utility.Vector2d;

/**
 * This is to normalize driving and make it easier for Mael to not drive into people. 
 * It also determines which driver controller/joystick should take priority and allows them to be used interchangeably
 */
public class DrivingProfiles extends SubsystemBase {

    private DoubleSupplier fowardControllerInput, strafeControllerInput, rotationControllerInput, throttleControllerInput;
    private DoubleSupplier fowardJoystickInput, strafeJoystickInput, rotationJoystickInput, throttleJoystickInput;
    private DoubleSupplier autoThrottleControllerInput, autoThrottleJoystickInput;
    private DoubleSupplier povInput;

    private double miniPower = 0.08;

    private boolean usePov = false;

    private double forwardOutput = 0, strafeOutput = 0, rotationOutput = 0;

    private double controllerDriveCurveMag, controllerTurnCurveMag;
    private double joystickDriveCurveMag, joystickTurnCurveMag;

    private boolean preferController = true;

    private final double ControllerDeadband = 0.05, JoystickDeadband = 0.13, AutoThrottleDeadband = 0.05;

    private double presetThrottleControl = 0.25;
    private boolean useThrottlePreset = false, autoAiming = false, autoDriving = false;

    private double autoForwardOutput = 0, autoStrafeOutput = 0, autoRotationOutput = 0;

    private boolean targetingLeftReef = false;
    private boolean lastSawObjectOnLeft = false;

    private Pose2d botPose = new Pose2d();;

    private int limelightPipelineIndex = 0;
    private int maxPipelineIndex = 9;


    public DrivingProfiles(boolean PreferController) {
        this.preferController = PreferController;

        if (Settings.tuningTelemetryEnabled) {
            SmartDashboard.putNumber("Tune Auto Aiming kP", AutoTargetingSettings.AutoAimPID.getP());
            SmartDashboard.putNumber("Tune Auto Aiming kI", AutoTargetingSettings.AutoAimPID.getI());
            SmartDashboard.putNumber("Tune Auto Aiming kD", AutoTargetingSettings.AutoAimPID.getD());

            SmartDashboard.putBoolean("Auto Aiming Enabled", AutoTargetingSettings.AutoAimingEnabled);
            SmartDashboard.putBoolean("Auto Driving Enabled", AutoTargetingSettings.AutoDrivingEnabled);
            SmartDashboard.putNumber("Auto Driving Power", AutoTargetingSettings.AutoDrivingPower);
            SmartDashboard.putNumber("Auto target percentage of blocked vision", AutoTargetingSettings.targetPercentageOfVisionBlocked);
        }

    }

    public DrivingProfiles() {
        new DrivingProfiles(preferController);
    }


    public void setUpControllerInputs(DoubleSupplier fowardInput, DoubleSupplier strafeInput, DoubleSupplier rotationInput, DoubleSupplier throttleInput, double driveCurveMag, double turnCurveMag) {
        this.fowardControllerInput = fowardInput;
        this.strafeControllerInput = strafeInput;
        this.rotationControllerInput = rotationInput;
        this.throttleControllerInput = throttleInput;
        this.controllerDriveCurveMag = driveCurveMag;
        this.controllerTurnCurveMag = turnCurveMag;
    }

    public void setUpJoystickInputs(DoubleSupplier fowardInput, DoubleSupplier strafeInput, DoubleSupplier rotationInput, DoubleSupplier throttleInput, double driveCurveMag, double turnCurveMag) {
        this.fowardJoystickInput = fowardInput;
        this.strafeJoystickInput = strafeInput;
        this.rotationJoystickInput = rotationInput;
        this.throttleJoystickInput = throttleInput;
        this.joystickDriveCurveMag = driveCurveMag;
        this.joystickTurnCurveMag = turnCurveMag;
    }

    public void setUpAutoThrottleControllerInput(DoubleSupplier autoThrottleControllerInput) {
        this.autoThrottleControllerInput = autoThrottleControllerInput;
    }

    public void setUpAutoThrottleJoystickInput(DoubleSupplier autoThrottleJoystickInput) {
        this.autoThrottleJoystickInput = autoThrottleJoystickInput;
    }

    public void setUpDpadControls(DoubleSupplier povInput) {
        this.povInput = povInput;
        usePov = true;
    }


    public void update() {
        if (preferController) {
            if (updateController());
            else if (updateJoystick());
            else stopDriving();
        } else {
            if (updateJoystick());
            else if (updateController());
            else stopDriving();
        }

        if (usePov && povInput.getAsDouble() >= 0) { // returns -1 when not pressed
            // pov is stupid: 0° is forward, 90° is right, 270° is left
            forwardOutput -= miniPower * Math.cos(Math.toRadians(povInput.getAsDouble()));
            strafeOutput -= miniPower * Math.sin(Math.toRadians(povInput.getAsDouble()));
        }

        if (autoAiming) updateAutoAiming();
        if (autoDriving) updateAutoDriving();
    }


    private boolean updateController() {
        double forward = fowardControllerInput.getAsDouble();
        double strafe = strafeControllerInput.getAsDouble();
        double turn = Functions.deadbandValue(rotationControllerInput.getAsDouble(), ControllerDeadband);
        double throttle = throttleControllerInput.getAsDouble();

        if (useThrottlePreset) throttle = presetThrottleControl;

        double Direction = Math.atan2(forward, strafe);
        double joystickPower = Functions.deadbandValue(Math.hypot(forward, strafe), ControllerDeadband);
        double drivePower = Functions.throttleCurve(joystickPower, controllerDriveCurveMag) * throttle;

        if (joystickPower == 0.0) drivePower = 0; // just to make sure

        forwardOutput = Math.cos(Direction) * drivePower;
        strafeOutput = Math.sin(Direction) * drivePower;
        rotationOutput = Functions.throttleCurve(turn, controllerTurnCurveMag) * throttle;

        return !(joystickPower == 0.0 && turn == 0.0); // returns true if in use
    }


    private boolean updateJoystick() {
        double forward = fowardJoystickInput.getAsDouble();
        double strafe = strafeJoystickInput.getAsDouble();
        double turn = Functions.deadbandValue(rotationJoystickInput.getAsDouble(), JoystickDeadband);
        double throttle = throttleJoystickInput.getAsDouble();

        if (useThrottlePreset) throttle = presetThrottleControl;

        double Direction = Math.atan2(forward, strafe);
        double joystickPower = Functions.deadbandValue(Math.hypot(forward, strafe), JoystickDeadband);
        double drivePower = Functions.throttleCurve(joystickPower, joystickDriveCurveMag) * throttle;

        if (joystickPower == 0.0) drivePower = 0; // just to make sure

        forwardOutput = Math.sin(Direction) * drivePower;
        strafeOutput = Math.cos(Direction) * drivePower;
        rotationOutput = Functions.throttleCurve(turn, joystickTurnCurveMag) * throttle;
        
        return !(joystickPower == 0.0 && turn == 0.0); // returns true if in use
    }


    private void updateAutoAiming() {
        

        if (LimelightHelpers.getTV("")) {
            double offsetTX = 0;
            if (targetingLeftReef) {
                offsetTX = AutoTargetingSettings.leftReefCrosshairOffset;
            }
            autoRotationOutput = Functions.minMaxValue(-1, 1, AutoTargetingSettings.AutoAimPID.calculate(LimelightHelpers.getTX("") + offsetTX, 0.0));
            
            if (AutoTargetingSettings.AutoAimingEnabled) rotationOutput = autoRotationOutput;

        } else if (AutoTargetingSettings.AutoAimingEnabled) {
            if (lastSawObjectOnLeft) rotationOutput = AutoTargetingSettings.searchingSpeed;
            else rotationOutput = -AutoTargetingSettings.searchingSpeed;
        }

    }

    private void updateAutoDriving() {

        Vector2d autoDrivingDirection = new Vector2d()
            .withMag(AutoTargetingSettings.AutoDrivingPower * (AutoTargetingSettings.targetPercentageOfVisionBlocked - LimelightHelpers.getTA("")))
            .withAngle(gyroData.yaw + LimelightHelpers.getTX("")); // in degrees

        double throttle = 0;
        if (preferController) {
            if (autoThrottleControllerInput.getAsDouble() > AutoThrottleDeadband) throttle = autoThrottleControllerInput.getAsDouble();
            else throttle = autoThrottleJoystickInput.getAsDouble();
        } else {
            if (autoThrottleJoystickInput.getAsDouble() > AutoThrottleDeadband) throttle = autoThrottleJoystickInput.getAsDouble();
            else throttle = autoThrottleControllerInput.getAsDouble();
        }


        autoForwardOutput = autoDrivingDirection.y * throttle;
        autoStrafeOutput = autoDrivingDirection.x * throttle;

        if (AutoTargetingSettings.AutoDrivingEnabled && LimelightHelpers.getTV("")) {
            forwardOutput += autoForwardOutput;
            strafeOutput += autoStrafeOutput;
        }
    }


    public void stopDriving() {
        forwardOutput = 0;
        strafeOutput = 0;
        rotationOutput = 0;
    }


    public double getForwardOutput() { return forwardOutput; }
    public double getStrafeOutput() { return strafeOutput; }
    public double getRotationOutput() { return rotationOutput; }


    public void enableSlowDown() { useThrottlePreset = true; }
    public void disableSlowDown() { useThrottlePreset = false; }

    public void enableAutoAim() { autoAiming = true; }
    public void disableAutoAim() { autoAiming = false; }
    public void enableAutoDriving() { autoDriving = true; }
    public void disableAutoDriving() { autoDriving = false; }

    public void setPipelineIndex(int index) { limelightPipelineIndex = index; }
    public int getPipelineIndex() { return limelightPipelineIndex; }
    public void increasePipelineIndex() {
        limelightPipelineIndex += 1; 
        if (limelightPipelineIndex > maxPipelineIndex) limelightPipelineIndex = 0;
    } 
    public void decreasePipelineIndex() {
        limelightPipelineIndex -= 1; 
        if (limelightPipelineIndex < 0) limelightPipelineIndex = maxPipelineIndex;
    } 

    @Override
    public void periodic() {

        LimelightHelpers.setPipelineIndex("", limelightPipelineIndex);

        if (LimelightHelpers.getTX("") > 0) lastSawObjectOnLeft = false;
        else if (LimelightHelpers.getTX("") < 0) lastSawObjectOnLeft = true;

        SmartDashboard.putNumber("Limelight pipeline", LimelightHelpers.getCurrentPipelineIndex(""));
        SmartDashboard.putNumber("Limelight TX", LimelightHelpers.getTX(""));
        SmartDashboard.putNumber("Limelight TA", LimelightHelpers.getTA(""));
        SmartDashboard.putBoolean("Limelight valid Target", LimelightHelpers.getTV(""));
        botPose = LimelightHelpers.getBotPose2d("");
        SmartDashboard.putString("Limelight getBotPose2d", botPose.toString());
        SmartDashboard.putString("Limelight drivetrain estimated pose", gyroData.robotPose.toString());

        SmartDashboard.putNumber("AUTO Targeting forward", autoForwardOutput);
        SmartDashboard.putNumber("AUTO Targeting strafe", autoStrafeOutput);
        SmartDashboard.putNumber("AUTO Targeting rotation", autoRotationOutput);

        SmartDashboard.putBoolean("Limelight last saw object on left", lastSawObjectOnLeft);

        //update settings
        if (Settings.tuningTelemetryEnabled) {
            AutoTargetingSettings.AutoAimPID.setP(SmartDashboard.getNumber("Tune Auto Aiming kP", AutoTargetingSettings.AutoAimPID.getP()));
            AutoTargetingSettings.AutoAimPID.setI(SmartDashboard.getNumber("Tune Auto Aiming kI", AutoTargetingSettings.AutoAimPID.getI()));
            AutoTargetingSettings.AutoAimPID.setD(SmartDashboard.getNumber("Tune Auto Aiming kD", AutoTargetingSettings.AutoAimPID.getD()));

            AutoTargetingSettings.AutoAimingEnabled = SmartDashboard.getBoolean("Auto Aiming Enabled", AutoTargetingSettings.AutoAimingEnabled);
            AutoTargetingSettings.AutoDrivingEnabled = SmartDashboard.getBoolean("Auto Driving Enabled", AutoTargetingSettings.AutoDrivingEnabled);
            AutoTargetingSettings.AutoDrivingPower = SmartDashboard.getNumber("Auto Driving Power", AutoTargetingSettings.AutoDrivingPower);
            AutoTargetingSettings.targetPercentageOfVisionBlocked = SmartDashboard.getNumber("Auto target percentage of blocked vision", AutoTargetingSettings.targetPercentageOfVisionBlocked);
        }

        SmartDashboard.putNumber("Pigeon yaw", gyroData.yaw);

        /*
        SmartDashboard.putNumber("Pigeon accel X", gyroData.accelX);
        SmartDashboard.putNumber("Pigeon accel Y", gyroData.accelY);
        SmartDashboard.putNumber("Pigeon accel Z", gyroData.accelZ);
        SmartDashboard.putNumber("Pigeon pitch", gyroData.pitch);
        SmartDashboard.putNumber("Pigeon roll", gyroData.roll);
        
        SmartDashboard.putNumber("Pigeon angVel X", gyroData.angVelX);
        SmartDashboard.putNumber("Pigeon angVel Y", gyroData.angVelY);
        SmartDashboard.putNumber("Pigeon angVel Z", gyroData.angVelZ);
        */
        
    }

}
