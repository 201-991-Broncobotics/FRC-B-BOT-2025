package frc.robot.subsystems;

import static frc.robot.Settings.CoralClawSettings.*;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.MotorConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;
import frc.robot.Constants.MotorConstants;
import frc.robot.Settings.CoralClawSettings;
import frc.robot.utility.CoralSystemPreset;
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
    private DoubleSupplier rencoder,lencoder; // gets angle in radians at the joint
    private double gearRatio = 1.0/3.0 * 1.0/5.0; // gear ratio from motor to each joint side

    private TalonFXConfiguration rightMotorConfig, leftMotorConfig;
    private SparkMaxConfig rollerMotorConfig;

    private double LeftDiffyTarget = 0, RightDiffyTarget = 0;

    private boolean enabled = false;

    public DiffyCoralClaw() {
        RollerPower = 0;

        TargetRoll = CoralClawSettings.startRoll;
        TargetPitch = CoralClawSettings.startPitch;

        // initialize motor
        lmotor = new TalonFX(MotorConstants.leftDiffyID);
        rmotor = new TalonFX(MotorConstants.rightDiffyID);
        rollerMotor = new SparkMax(MotorConstants.coralRollerID, MotorType.kBrushless);

        // configure motors
        lmotor.setNeutralMode(NeutralModeValue.Brake);
        rmotor.setNeutralMode(NeutralModeValue.Brake);

        rollerMotorConfig = new SparkMaxConfig();
        rollerMotorConfig.idleMode(IdleMode.kCoast);
        rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // setup absolute encoders
        lThroughBore = new ThroughBoreEncoder(0, 1, 2); // Digital ports on the RobotRio
        rThroughBore = new ThroughBoreEncoder(3, 4, 5);

        // Encoder object created to display position values
        rencoder = () -> -1 * gearRatio * rmotor.getPosition().getValueAsDouble() * 2*Math.PI; // in radians
        lencoder = () -> gearRatio * lmotor.getPosition().getValueAsDouble() * 2*Math.PI;

        if (Settings.tuningTelemetryEnabled) {

            SmartDashboard.putNumber("Tune Left Coral Claw kP", CoralClawSettings.LeftDiffyPID.getP());
            SmartDashboard.putNumber("Tune Left Coral Claw kI", CoralClawSettings.LeftDiffyPID.getI());
            SmartDashboard.putNumber("Tune Left Coral Claw kD", CoralClawSettings.LeftDiffyPID.getD());
            SmartDashboard.putNumber("Tune Right Coral Claw kP", CoralClawSettings.RightDiffyPID.getP());
            SmartDashboard.putNumber("Tune Right Coral Claw kI", CoralClawSettings.RightDiffyPID.getI());
            SmartDashboard.putNumber("Tune Right Coral Claw kD", CoralClawSettings.RightDiffyPID.getD());

            SmartDashboard.putNumber("Testing Diffy Pitch", CoralClawSettings.testingPitch);
            SmartDashboard.putNumber("Testing Diffy Roll", CoralClawSettings.testingRoll);
        }
    
    }


    public void update() {
        setDiffyClaw(CoralClawSettings.testingPitch, CoralClawSettings.testingRoll);

        //Limits
        TargetPitch = Functions.minMaxValue(minPitch, maxPitch, TargetPitch);
        TargetRoll = Functions.minMaxValue(-0.5*rollRange, 0.5*rollRange, TargetRoll);

        // find motor target positions
        LeftDiffyTarget = TargetPitch-(TargetRoll*0.5);
        RightDiffyTarget = TargetPitch+(TargetRoll*0.5);

        if (enabled) {
            lmotor.set(CoralClawSettings.LeftDiffyPID.calculate(lencoder.getAsDouble(), LeftDiffyTarget) + CoralClawSettings.gravityPower * Math.sin(getCurrentPitch()));
            rmotor.set(-1 * (CoralClawSettings.RightDiffyPID.calculate(rencoder.getAsDouble(), RightDiffyTarget) + CoralClawSettings.gravityPower * Math.sin(getCurrentPitch())));
        } else {
            lmotor.set(0);
            rmotor.set(0);
        }

        rollerMotor.set(RollerPower);
        
    }



    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // read PID coefficients from SmartDashboard
        if (Settings.tuningTelemetryEnabled) {

            CoralClawSettings.LeftDiffyPID.setP(SmartDashboard.getNumber("Tune Left Coral Claw kP", CoralClawSettings.LeftDiffyPID.getP()));
            CoralClawSettings.LeftDiffyPID.setI(SmartDashboard.getNumber("Tune Left Coral Claw kI", CoralClawSettings.LeftDiffyPID.getI()));
            CoralClawSettings.LeftDiffyPID.setD(SmartDashboard.getNumber("Tune Left Coral Claw kD", CoralClawSettings.LeftDiffyPID.getD()));
            CoralClawSettings.RightDiffyPID.setP(SmartDashboard.getNumber("Tune Right Coral Claw kP", CoralClawSettings.RightDiffyPID.getP()));
            CoralClawSettings.RightDiffyPID.setI(SmartDashboard.getNumber("Tune Right Coral Claw kI", CoralClawSettings.RightDiffyPID.getI()));
            CoralClawSettings.RightDiffyPID.setD(SmartDashboard.getNumber("Tune Right Coral Claw kD", CoralClawSettings.RightDiffyPID.getD()));

            CoralClawSettings.testingPitch = SmartDashboard.getNumber("Testing Diffy Pitch", CoralClawSettings.testingPitch);
            CoralClawSettings.testingRoll = SmartDashboard.getNumber("Testing Diffy Roll", CoralClawSettings.testingRoll);
        }

        SmartDashboard.putNumber("CoralClaw Target Pitch", Math.toDegrees(TargetPitch));
        SmartDashboard.putNumber("CoralClaw Target Roll", Math.toDegrees(TargetRoll));
        SmartDashboard.putNumber("CoralClaw Current Pitch", Math.toDegrees(getCurrentPitch()));
        SmartDashboard.putNumber("CoralClaw Current Roll", Math.toDegrees(getCurrentRoll()));
        SmartDashboard.putNumber("CoralClaw Right Motor Power", rmotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("CoralClaw Left Motor Power", lmotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("CoralClaw Right Motor Position", rmotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("CoralClaw Left Motor Position", lmotor.getPosition().getValueAsDouble());

        SmartDashboard.putNumber("CoralClaw Roller Power", RollerPower);
        SmartDashboard.putNumber("Coral Roller Current", rollerMotor.getOutputCurrent());


        SmartDashboard.putNumber("Right ThroughBore Relative Raw", rThroughBore.getRelativeRaw());
        SmartDashboard.putNumber("Right ThroughBore Absolute Raw", rThroughBore.getAbsoluteRaw());
        SmartDashboard.putNumber("Right ThroughBore Relative Angle", rThroughBore.getRelativeAngle());
        SmartDashboard.putNumber("Right ThroughBore Absolute Angle", rThroughBore.getAbsoluteAngle());

        SmartDashboard.putNumber("Left ThroughBore Relative Angle", lThroughBore.getRelativeAngle());
        SmartDashboard.putNumber("Left ThroughBore Absolute Angle", lThroughBore.getAbsoluteAngle());

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
        return rencoder.getAsDouble() - lencoder.getAsDouble();
    }

    public void stopRoller() { RollerPower = 0; }
    public void intakeRoller() { RollerPower = CoralClawSettings.intakePower; }
    public void outtakeRoller() { RollerPower = CoralClawSettings.outtakePower; }
    public void holdRoller() { RollerPower = CoralClawSettings.holdPower; }

    public void setDiffyClaw(double pitch, double roll) {
        TargetPitch = pitch;
        TargetRoll = roll;
    }

    public void Enable() { enabled = true; }
    public void Disable() { enabled = false; }
    public void toggleEnabled() { enabled = !enabled; }


    public void goToPreset(CoralSystemPreset coralSystemPreset) {
        setDiffyClaw(coralSystemPreset.clawPitch, coralSystemPreset.clawRoll);
    }

}
