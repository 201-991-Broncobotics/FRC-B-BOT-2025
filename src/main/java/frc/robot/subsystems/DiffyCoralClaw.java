package frc.robot.subsystems;

import static frc.robot.Settings.CoralClawSettings.*;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Amps;
import static frc.robot.Constants.MotorConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;
import frc.robot.Constants.MotorConstants;
import frc.robot.Settings.CoralClawSettings;
import frc.robot.Settings.CoralSystemPresets;
import frc.robot.Settings.CoralSystemSettings;
import frc.robot.utility.CoralSystemPreset;
import frc.robot.utility.ElapsedTime;
import frc.robot.utility.ElapsedTime.Resolution;
import frc.robot.utility.Functions;
import frc.robot.utility.ThroughBoreEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class DiffyCoralClaw extends SubsystemBase {

    //Targeted Pos for each motor
    double rMotorPos = 0.0; //Current Diffy Position for Right motor
    double lMotorPos = 0.0; //Current Diffy Position for Left Motor

    double TargetRoll, TargetPitch;

    private double RollerPower;

    private ThroughBoreEncoder lThroughBore, rThroughBore;

    private TalonFX lmotor, rmotor;
    private SparkMax rollerMotor;
    private StatusCode lMotorConfig, rMotorConfig;
    private CurrentLimitsConfigs currentLimitConfigs;

    private DoubleSupplier rencoder,lencoder; // gets angle in radians at the joint
    private double gearRatio = 1.0/3.0 * 1.0/5.0; // gear ratio from motor to each joint side

    private TalonFXConfiguration rightMotorConfig, leftMotorConfig;
    private SparkMaxConfig rollerMotorConfig;

    private double LeftDiffyTarget = 0, RightDiffyTarget = 0;

    private ElapsedTime runTime;
    private double frameTime = 0;

    private DoubleSupplier ManualControlAxis1, ManualControlAxis2;
    private boolean GoToPosition = false; // whether or not the manual control controls the power up and down or sets the position

    private boolean snapToPreset = true;
    private double presetTolerance = Math.toRadians(45);

    private boolean overrideManualControl = false;
    private double lastManualControl1 = 0, lastManualControl2 = 0;
    private double overrideOverrideTolerance = 0.1; // units are joystick input

    private boolean enabled = true;

    public DiffyCoralClaw() {
        RollerPower = 0;

        runTime = new ElapsedTime(Resolution.SECONDS);

        TargetRoll = CoralClawSettings.startRoll;
        TargetPitch = CoralClawSettings.startPitch;

        // initialize motor
        lmotor = new TalonFX(MotorConstants.leftDiffyID);
        rmotor = new TalonFX(MotorConstants.rightDiffyID);
        rollerMotor = new SparkMax(MotorConstants.coralRollerID, MotorType.kBrushless);

        // configure motors
        lmotor.setNeutralMode(NeutralModeValue.Brake);
        rmotor.setNeutralMode(NeutralModeValue.Brake);
        
        // Falcon current limits so we don't break the super weak tiny 3d printed pulleys that someone thought was a good idea
        currentLimitConfigs = new CurrentLimitsConfigs()
             .withStatorCurrentLimit(Amps.of(CoralClawSettings.DiffyMotorCurrentLimit))
             .withStatorCurrentLimitEnable(true);
             
        lMotorConfig = lmotor.getConfigurator().apply(currentLimitConfigs);
        rMotorConfig = rmotor.getConfigurator().apply(currentLimitConfigs);


        rollerMotorConfig = new SparkMaxConfig();
        rollerMotorConfig.idleMode(IdleMode.kCoast);
        rollerMotorConfig.smartCurrentLimit(CoralClawSettings.RollerCurrentLimit);
        rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // setup absolute encoders
        lThroughBore = new ThroughBoreEncoder(0, 1, 2); // Digital ports on the RobotRio
        rThroughBore = new ThroughBoreEncoder(3, 4, 5);

        lThroughBore.resetRelative();
        lThroughBore.setAbsoluteZero(Math.toRadians(240.528966));
        rThroughBore.resetRelative();
        rThroughBore.setAbsoluteZero(Math.toRadians(237.302106));

        lThroughBore.setRelativeZero(lThroughBore.getAbsoluteAngleNorm());
        rThroughBore.setRelativeZero(rThroughBore.getAbsoluteAngleNorm());

        // flip negatives for absolute
        // Encoder object created to display position values
        rencoder = () -> rThroughBore.getRelativeAngle(); // in radians // -1 * gearRatio * rmotor.getPosition().getValueAsDouble() * 2*Math.PI
        lencoder = () -> -lThroughBore.getRelativeAngle(); // gearRatio * lmotor.getPosition().getValueAsDouble() * 2*Math.PI

        if (Settings.tuningTelemetryEnabled) {

            SmartDashboard.putNumber("Tune Left Coral Claw kP", CoralClawSettings.LeftDiffyPID.getP());
            SmartDashboard.putNumber("Tune Left Coral Claw kI", CoralClawSettings.LeftDiffyPID.getI());
            SmartDashboard.putNumber("Tune Left Coral Claw kD", CoralClawSettings.LeftDiffyPID.getD());
            SmartDashboard.putNumber("Tune Right Coral Claw kP", CoralClawSettings.RightDiffyPID.getP());
            SmartDashboard.putNumber("Tune Right Coral Claw kI", CoralClawSettings.RightDiffyPID.getI());
            SmartDashboard.putNumber("Tune Right Coral Claw kD", CoralClawSettings.RightDiffyPID.getD());

            //SmartDashboard.putNumber("Testing Diffy Pitch", CoralClawSettings.testingPitch);
            //SmartDashboard.putNumber("Testing Diffy Roll", CoralClawSettings.testingRoll);
        }

        frameTime = runTime.time();
        runTime.reset();
    
    }


    public void update() {
        // setDiffyClaw(CoralClawSettings.testingPitch, CoralClawSettings.testingRoll);

        if (ManualControlAxis1 != null && ManualControlAxis2 != null) {
            if (!GoToPosition) {
                if ((Math.abs(ManualControlAxis1.getAsDouble()) > overrideOverrideTolerance || Math.abs(ManualControlAxis2.getAsDouble()) > overrideOverrideTolerance) && overrideManualControl) overrideManualControl = false;
                if (!overrideManualControl) {
                    TargetPitch += ManualControlAxis1.getAsDouble() * CoralClawSettings.manualPitchSpeed * frameTime;
                    TargetRoll += ManualControlAxis2.getAsDouble() * CoralClawSettings.manualRollSpeed * frameTime;
                } 
            } else {
                if ((Math.abs(ManualControlAxis1.getAsDouble() - lastManualControl1) > overrideOverrideTolerance || Math.abs(ManualControlAxis2.getAsDouble() - lastManualControl2) > overrideOverrideTolerance) && overrideManualControl) overrideManualControl = false;
                if (!overrideManualControl) {
                    TargetPitch = ManualControlAxis1.getAsDouble() * (maxPitch - minPitch) + minPitch;
                    TargetRoll = ManualControlAxis2.getAsDouble() * (0.5*rollRange - -0.5*rollRange) + -0.5*rollRange;
                    if (snapToPreset) {
                        if (Math.abs(TargetRoll - Math.toRadians(0)) < presetTolerance) TargetRoll = Math.toRadians(0);
                        else if (Math.abs(TargetRoll - Math.toRadians(90)) < presetTolerance) TargetRoll = Math.toRadians(90);
                        else if (Math.abs(TargetRoll - Math.toRadians(-90)) < presetTolerance) TargetRoll = Math.toRadians(-90);
                    }
                }
            }
        }

        //Limits
        TargetPitch = Functions.minMaxValue(minPitch, maxPitch, TargetPitch);
        TargetRoll = Functions.minMaxValue(-0.5*rollRange, 0.5*rollRange, TargetRoll);

        // find motor target positions
        LeftDiffyTarget = TargetPitch - (TargetRoll * CoralClawSettings.rollAngleExaggeration); 
        RightDiffyTarget = TargetPitch + (TargetRoll * CoralClawSettings.rollAngleExaggeration); 

        //RightDiffyTarget = Functions.minMaxValue(-110, 180, RightDiffyTarget);
        //LeftDiffyTarget = Functions.minMaxValue(-125, 110, LeftDiffyTarget);

        if (enabled) {
            rmotor.set(-1 * (CoralClawSettings.LeftDiffyPID.calculate(lencoder.getAsDouble(), LeftDiffyTarget) + CoralClawSettings.gravityPower * Math.sin(getCurrentPitch())));
            lmotor.set((CoralClawSettings.RightDiffyPID.calculate(rencoder.getAsDouble(), RightDiffyTarget) + CoralClawSettings.gravityPower * Math.sin(getCurrentPitch())));
        } else {
            lmotor.set(0);
            rmotor.set(0);
        }

        rollerMotor.set(-RollerPower);
        
    }



    @Override
    public void periodic() {
        frameTime = runTime.time();
        runTime.reset();

        // This method will be called once per scheduler run
        // read PID coefficients from SmartDashboard
        if (Settings.tuningTelemetryEnabled) {

            CoralClawSettings.LeftDiffyPID.setP(SmartDashboard.getNumber("Tune Left Coral Claw kP", CoralClawSettings.LeftDiffyPID.getP()));
            CoralClawSettings.LeftDiffyPID.setI(SmartDashboard.getNumber("Tune Left Coral Claw kI", CoralClawSettings.LeftDiffyPID.getI()));
            CoralClawSettings.LeftDiffyPID.setD(SmartDashboard.getNumber("Tune Left Coral Claw kD", CoralClawSettings.LeftDiffyPID.getD()));
            CoralClawSettings.RightDiffyPID.setP(SmartDashboard.getNumber("Tune Right Coral Claw kP", CoralClawSettings.RightDiffyPID.getP()));
            CoralClawSettings.RightDiffyPID.setI(SmartDashboard.getNumber("Tune Right Coral Claw kI", CoralClawSettings.RightDiffyPID.getI()));
            CoralClawSettings.RightDiffyPID.setD(SmartDashboard.getNumber("Tune Right Coral Claw kD", CoralClawSettings.RightDiffyPID.getD()));

            //CoralClawSettings.testingPitch = SmartDashboard.getNumber("Testing Diffy Pitch", CoralClawSettings.testingPitch);
            //CoralClawSettings.testingRoll = SmartDashboard.getNumber("Testing Diffy Roll", CoralClawSettings.testingRoll);
        }

        SmartDashboard.putNumber("CoralClaw Target Pitch", Math.toDegrees(TargetPitch));
        SmartDashboard.putNumber("CoralClaw Target Roll", Math.toDegrees(TargetRoll));
        SmartDashboard.putNumber("CoralClaw Current Pitch", Math.toDegrees(getCurrentPitch()));
        SmartDashboard.putNumber("CoralClaw Current Roll", Math.toDegrees(getCurrentRoll()));
        //SmartDashboard.putNumber("CoralClaw Right Motor Power", rmotor.getMotorVoltage().getValueAsDouble());
        //SmartDashboard.putNumber("CoralClaw Left Motor Power", lmotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("CoralClaw Right Motor Target", Math.toDegrees(RightDiffyTarget));
        SmartDashboard.putNumber("CoralClaw Left Motor Target", Math.toDegrees(LeftDiffyTarget));
        //SmartDashboard.putNumber("CoralClaw Right Motor Position", rmotor.getPosition().getValueAsDouble());
        //SmartDashboard.putNumber("CoralClaw Left Motor Position", lmotor.getPosition().getValueAsDouble());

        // SmartDashboard.putNumber("CoralClaw Roller Power", RollerPower);
        SmartDashboard.putNumber("Coral Roller Current", rollerMotor.getOutputCurrent());

        SmartDashboard.putNumber("CoralClaw Right Encoder Position", Math.toDegrees(rencoder.getAsDouble()));
        SmartDashboard.putNumber("CoralClaw Left Encoder Position", Math.toDegrees(lencoder.getAsDouble()));


        SmartDashboard.putNumber("Right ThroughBore Absolute Angle", Math.toDegrees(rThroughBore.getAbsoluteAngleNorm()));
        SmartDashboard.putNumber("Left ThroughBore Absolute Angle", Math.toDegrees(lThroughBore.getAbsoluteAngleNorm()));

    }

    /**
     * Gets current pitch in radians
     */
    public double getCurrentPitch() {
        return 0.5 * (rencoder.getAsDouble() + lencoder.getAsDouble());
    }
    /**
     * Gets current roll in radians
     */
    public double getCurrentRoll() {
        return (rencoder.getAsDouble() - lencoder.getAsDouble()) * 0.5;
    }

    public void stopRoller() { RollerPower = 0; }
    public void intakeRoller() { RollerPower = CoralClawSettings.intakePower; }
    public void outtakeRoller() { RollerPower = CoralClawSettings.outtakePower; }
    public void holdRoller() { RollerPower = CoralClawSettings.holdPower; }

    public void setDiffyClaw(double pitch, double roll) {
        TargetPitch = pitch;
        TargetRoll = roll;

        overrideManualControl = true;
        if (GoToPosition) { 
            lastManualControl1 = ManualControlAxis1.getAsDouble();
            lastManualControl2 = ManualControlAxis2.getAsDouble();
        }
    }

    public void Enable() { enabled = true; }
    public void Disable() { enabled = false; }
    public void toggleEnabled() { enabled = !enabled; }


    public void goToPreset(CoralSystemPreset coralSystemPreset) {
        setDiffyClaw(coralSystemPreset.clawPitch, coralSystemPreset.clawRoll);

        overrideManualControl = true;
        if (GoToPosition) { 
            lastManualControl1 = ManualControlAxis1.getAsDouble();
            lastManualControl2 = ManualControlAxis2.getAsDouble();
        }
    }

    public void setManualControl(DoubleSupplier controlAxis1, DoubleSupplier controlAxis2, boolean goToPosition) {
        ManualControlAxis1 = controlAxis1;
        ManualControlAxis2 = controlAxis2;
        GoToPosition = goToPosition;
    }

    public void setManualControl(DoubleSupplier controlAxis1, DoubleSupplier controlAxis2) {
        setManualControl(controlAxis1, controlAxis2, false);
    }

    public void goToElevatorPreset() {
        switch (CoralElevatorSystem.ElevatorStage) {
            case 0: goToPreset(CoralSystemPresets.GroundIntake); break;
            case 1: goToPreset(CoralSystemPresets.L1Reef); break;
            case 2: goToPreset(CoralSystemPresets.CoralStationIntake); break;
            case 3: goToPreset(CoralSystemPresets.L2Reef); break;
            case 4: goToPreset(CoralSystemPresets.L3Reef); break;
            case 5: goToPreset(CoralSystemPresets.L4Reef); break;
        }

        overrideManualControl = true;
        if (GoToPosition) { 
            lastManualControl1 = ManualControlAxis1.getAsDouble();
            lastManualControl2 = ManualControlAxis2.getAsDouble();
        }
    }


    public void switchRotation() {
        if (TargetRoll > Math.toRadians(45)) TargetRoll = Math.toRadians(0);
        else if (TargetRoll > Math.toRadians(-45)) TargetRoll = Math.toRadians(-90);
        else TargetRoll = Math.toRadians(90);
        
    }


    public boolean combinedElevatorCoralPitchControl(double input, double minAngle, double maxAngle) {
        if ((TargetPitch < maxAngle && input > 0) || (TargetPitch > minAngle && input < 0)) {
            TargetPitch += input * CoralClawSettings.manualPitchSpeed * frameTime;
            TargetPitch = Functions.minMaxValue(minAngle, maxAngle, TargetPitch);
            return false; // don't need to move the elevator yet
        }
        return true; // need to move the elevator instead
    }


}
