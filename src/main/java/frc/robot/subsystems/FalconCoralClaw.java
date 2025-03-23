package frc.robot.subsystems;

import static frc.robot.Settings.CoralClawSettings.*;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.MotorConstants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings;
import frc.robot.Constants.MotorConstants;
import frc.robot.Settings.CoralClawSettings;
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

public class FalconCoralClaw extends SubsystemBase {

    //Targeted Pos for each motor
    double rMotorPos = 0.0; //Current Diffy Position for Right motor
    double lMotorPos = 0.0; //Current Diffy Position for Left Motor

    double Roll, Pitch;

    private double RollerPower;

    private TalonFX lmotor, rmotor;
    private SparkMax rollerMotor;
    //private SparkMax rmotor; // this is separate so I can see where to comment out code
    private DoubleSupplier rencoder,lencoder; // gets angle in radians at the joint
    private double gearRatio = 1.0/3.0 * 1.0/5.0; // gear ratio from motor to each joint side

    private TalonFXConfiguration rightMotorConfig, leftMotorConfig;
    private SparkMaxConfig rollerMotorConfig;

    private double LeftDiffyTarget = 0, RightDiffyTarget = 0;

    private boolean enabled = false;

    public FalconCoralClaw() {
        RollerPower = 0;

        Roll = CoralClawSettings.startRoll;
        Pitch = CoralClawSettings.startPitch;

        // initialize motor
        lmotor = new TalonFX(MotorConstants.leftDiffyID);
        rmotor = new TalonFX(MotorConstants.rightDiffyID);
        rollerMotor = new SparkMax(MotorConstants.coralRollerID, MotorType.kBrushless);

        // create configurations
        rollerMotorConfig = new SparkMaxConfig();
        rollerMotorConfig.idleMode(IdleMode.kCoast);

        lmotor.setNeutralMode(NeutralModeValue.Brake);
        rmotor.setNeutralMode(NeutralModeValue.Brake);
        rollerMotor.configure(rollerMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Encoder object created to display position values - I think it uses internal encoder or smth?
        rencoder = () -> gearRatio * 0 * 2*Math.PI; // converts to radians // rmotor.getEncoder().getPosition()
        lencoder = () -> gearRatio * lmotor.getPosition().getValueAsDouble() * 2*Math.PI;

        if (Settings.tuningTelemetryEnabled) {
            //SmartDashboard.putNumber("Tune Coral Claw kP", CoralClawSettings.kP);
            //SmartDashboard.putNumber("Tune Coral Claw kI", CoralClawSettings.kI);
            //SmartDashboard.putNumber("Tune Coral Claw kD", CoralClawSettings.kD);
            //SmartDashboard.putNumber("Tune Coral Claw Right FF", CoralClawSettings.RFF);
            //SmartDashboard.putNumber("Tune Coral Claw Left FF", CoralClawSettings.LFF);

            SmartDashboard.putNumber("Tune Left Coral Claw kP", CoralClawSettings.LeftDiffyPID.getP());
            SmartDashboard.putNumber("Tune Left Coral Claw kI", CoralClawSettings.LeftDiffyPID.getI());
            SmartDashboard.putNumber("Tune Left Coral Claw kD", CoralClawSettings.LeftDiffyPID.getD());
            SmartDashboard.putNumber("Tune Right Coral Claw kP", CoralClawSettings.RightDiffyPID.getP());
            SmartDashboard.putNumber("Tune Right Coral Claw kI", CoralClawSettings.RightDiffyPID.getI());
            SmartDashboard.putNumber("Tune Right Coral Claw kD", CoralClawSettings.RightDiffyPID.getD());

            //SmartDashboard.putNumber("Tune Coral Roller SmartCurrent", CoralClawSettings.intakeSmartStallCurrent);
            //SmartDashboard.putNumber("Tune Coral Roller SecondaryCurrent", CoralClawSettings.intakeSecondaryCurrent);
        }

        SmartDashboard.putNumber("Testing Diffy Pitch", CoralClawSettings.testingPitch);
        SmartDashboard.putNumber("Testing Diffy Roll", CoralClawSettings.testingRoll);
    
    }


    public void update() {
        setDiffyClaw(CoralClawSettings.testingPitch, CoralClawSettings.testingRoll);

        //Limits
        if(Pitch > maxPitch) {Pitch = maxPitch;} else if (Pitch < minPitch){Pitch = minPitch;}
        if(Roll > 0.5*rollRange) {Roll = 0.5*rollRange;} else if (Roll < -0.5*rollRange){Roll = -0.5*rollRange;}

        rollerMotor.set(RollerPower);

        calculateTargetPos();//get angles for each motor

        //rmotor.getClosedLoopController().setReference(rMotorPos/360.0, SparkMax.ControlType.kPosition);//convert 0-360 to 0-1 & set Motor
        //lmotor.getClosedLoopController().setReference(lMotorPos/360.0, SparkMax.ControlType.kPosition);//convert 0-360 to 0-1 & set Motor
        if (enabled) {
            lmotor.set(CoralClawSettings.LeftDiffyPID.calculate(lencoder.getAsDouble(), LeftDiffyTarget));
            rmotor.set(CoralClawSettings.RightDiffyPID.calculate(lencoder.getAsDouble(), RightDiffyTarget));
        } else {
            lmotor.set(0);
            rmotor.set(0);
        }
        
    }



    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // read PID coefficients from SmartDashboard
        if (Settings.tuningTelemetryEnabled) {
            //CoralClawSettings.kP = SmartDashboard.getNumber("Tune Coral Claw kP", CoralClawSettings.kP);
            //CoralClawSettings.kI = SmartDashboard.getNumber("Tune Coral Claw kI", CoralClawSettings.kI);
            //CoralClawSettings.kD = SmartDashboard.getNumber("Tune Coral Claw kD", CoralClawSettings.kD);
            //CoralClawSettings.RFF = SmartDashboard.getNumber("Tune Coral Claw Right FF", CoralClawSettings.RFF);
            //CoralClawSettings.LFF = SmartDashboard.getNumber("Tune Coral Claw Left FF", CoralClawSettings.LFF);

            CoralClawSettings.LeftDiffyPID.setP(SmartDashboard.getNumber("Tune Left Coral Claw kP", CoralClawSettings.LeftDiffyPID.getP()));
            CoralClawSettings.LeftDiffyPID.setI(SmartDashboard.getNumber("Tune Left Coral Claw kI", CoralClawSettings.LeftDiffyPID.getI()));
            CoralClawSettings.LeftDiffyPID.setD(SmartDashboard.getNumber("Tune Left Coral Claw kD", CoralClawSettings.LeftDiffyPID.getD()));
            CoralClawSettings.RightDiffyPID.setP(SmartDashboard.getNumber("Tune Right Coral Claw kP", CoralClawSettings.RightDiffyPID.getP()));
            CoralClawSettings.RightDiffyPID.setI(SmartDashboard.getNumber("Tune Right Coral Claw kI", CoralClawSettings.RightDiffyPID.getI()));
            CoralClawSettings.RightDiffyPID.setD(SmartDashboard.getNumber("Tune Right Coral Claw kD", CoralClawSettings.RightDiffyPID.getD()));

            //CoralClawSettings.intakeSmartStallCurrent = (int) SmartDashboard.getNumber("Tune Coral Roller SmartCurrent", CoralClawSettings.intakeSmartStallCurrent);
            //CoralClawSettings.intakeSecondaryCurrent = (int) SmartDashboard.getNumber("Tune Coral Roller SecondaryCurrent", CoralClawSettings.intakeSecondaryCurrent);
        }

        CoralClawSettings.testingPitch = SmartDashboard.getNumber("Testing Diffy Pitch", CoralClawSettings.testingPitch);
        CoralClawSettings.testingRoll = SmartDashboard.getNumber("Testing Diffy Roll", CoralClawSettings.testingRoll);
        SmartDashboard.putNumber("Coral Roller Current", rollerMotor.getOutputCurrent());

        //rightMotorConfig.closedLoop.pidf(CoralClawSettings.kP, CoralClawSettings.kI, CoralClawSettings.kD, CoralClawSettings.RFF);
        //leftMotorConfig.closedLoop.pidf(CoralClawSettings.kP, CoralClawSettings.kI, CoralClawSettings.kD, CoralClawSettings.LFF);

        //rmotor.configureAsync(rightMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        //lmotor.configureAsync(leftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        //rollerMotor.configureAsync(coralMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SmartDashboard.putNumber("CoralClaw Target Pitch", Pitch);
        SmartDashboard.putNumber("CoralClaw Target Roll", Roll);
        SmartDashboard.putNumber("CoralClaw Current Pitch", getCurrentPitch());
        SmartDashboard.putNumber("CoralClaw Current Roll", getCurrentRoll());
        SmartDashboard.putNumber("CoralClaw Right Motor Power", rmotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("CoralClaw Left Motor Power", lmotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("CoralClaw Right Motor Position", rmotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("CoralClaw Left Motor Position", lmotor.getPosition().getValueAsDouble());

        SmartDashboard.putNumber("CoralClaw Roller Power", RollerPower);

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void calculateTargetPos(){
        lMotorPos = Pitch-(Roll/2.0);
        rMotorPos = Pitch+(Roll/2.0);
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

    public void stopRoller() {
        RollerPower = 0;
    }

    public void intakeRoller() {
        RollerPower = CoralClawSettings.intakePower;
    }

    public void outtakeRoller() {
        RollerPower = CoralClawSettings.outtakePower;
    }

    public void holdRoller() {
        RollerPower = CoralClawSettings.holdPower;
    }

    public void setDiffyClaw(double pitch, double roll) {
        Pitch = pitch;
        Roll = roll;
    }

    public void toggleLeftDiffyPosition() {
        if (CoralClawSettings.startRotatePosition == LeftDiffyTarget) LeftDiffyTarget = CoralClawSettings.secondRotatePosition;
        else LeftDiffyTarget = CoralClawSettings.startRotatePosition;
    }

    public void Enable() { enabled = true; }
    public void Disable() { enabled = false; }
    public void toggleEnabled() { enabled = !enabled; }

}
