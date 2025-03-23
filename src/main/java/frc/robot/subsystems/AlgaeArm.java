package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.MotorConstants;
import frc.robot.Settings.AlgaeArmSettings;
import frc.robot.Settings.CoralClawSettings;
import frc.robot.utility.Functions;

public class AlgaeArm extends SubsystemBase {
    
    private TalonFX pivotMotor;
    private SparkFlex algaeRoller;
    private SparkFlexConfig algaeRollerConfig;

    private DoubleSupplier algaePivotEncoder;

    private double PivotMotorPower, AlgaeRollerPower;

    private double TargetPivotAngle;

    private final double pivotGearRatio = 1.0/5.0 * 1.0/5.0;

    private double pivotZeroAngle = 0;

    private boolean enabled = false;


    public AlgaeArm() {
        
        algaeRollerConfig = new SparkFlexConfig();

        TargetPivotAngle = AlgaeArmSettings.AlgaePivotStartAngle;


        pivotMotor = new TalonFX(MotorConstants.algaePivotID);
        pivotMotor.setNeutralMode(NeutralModeValue.Coast);
        algaeRoller = new SparkFlex(MotorConstants.algaeRollerID, MotorType.kBrushless);
        algaeRollerConfig.idleMode(IdleMode.kCoast);
        algaeRollerConfig.smartCurrentLimit(AlgaeArmSettings.algaeRollerStallCurrent);
        algaeRoller.configure(algaeRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        PivotMotorPower = 0;
        AlgaeRollerPower = 0;

        pivotZeroAngle = pivotMotor.getPosition().getValueAsDouble();

        algaePivotEncoder = () -> (pivotMotor.getPosition().getValueAsDouble() - pivotZeroAngle) * pivotGearRatio * 2 * Math.PI + AlgaeArmSettings.AlgaePivotStartAngle;


        SmartDashboard.putNumber("Tune Algae Pivot kP", AlgaeArmSettings.AlgaePivotPID.getP());
        SmartDashboard.putNumber("Tune Algae Pivot kI", AlgaeArmSettings.AlgaePivotPID.getI());
        SmartDashboard.putNumber("Tune Algae Pivot kD", AlgaeArmSettings.AlgaePivotPID.getD());

    }


    public void update() {

        TargetPivotAngle = Functions.minMaxValue(AlgaeArmSettings.MinPivotAngle, AlgaeArmSettings.MaxPivotAngle, TargetPivotAngle);

        PivotMotorPower = AlgaeArmSettings.AlgaePivotPID.calculate(algaePivotEncoder.getAsDouble(), TargetPivotAngle);

        if (algaeRoller.getOutputCurrent() > AlgaeArmSettings.algaeRollerHasIntakedCurrent && AlgaeRollerPower == AlgaeArmSettings.IntakePower) {
            AlgaeRollerPower = AlgaeArmSettings.HoldPower;
        }

        if (enabled) {
            pivotMotor.set(PivotMotorPower);
        } else {
            pivotMotor.set(0);
        }

        algaeRoller.set(-AlgaeRollerPower);

    }

    @Override
    public void periodic() { 
        SmartDashboard.putNumber("Algae Current Pivot Angle", Math.toDegrees(algaePivotEncoder.getAsDouble()));
        SmartDashboard.putNumber("Algae Target Pivot Angle", Math.toDegrees(TargetPivotAngle));
        SmartDashboard.putNumber("Algae Pivot Motor Current", pivotMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Algae Pivot Motor Power", PivotMotorPower);

        SmartDashboard.putNumber("Algae Roller Power", AlgaeRollerPower);
        SmartDashboard.putNumber("Algae Roller Motor Current", algaeRoller.getOutputCurrent());


        AlgaeArmSettings.AlgaePivotPID.setP(SmartDashboard.getNumber("Tune Algae Pivot kP", AlgaeArmSettings.AlgaePivotPID.getP()));
        AlgaeArmSettings.AlgaePivotPID.setI(SmartDashboard.getNumber("Tune Algae Pivot kI", AlgaeArmSettings.AlgaePivotPID.getI()));
        AlgaeArmSettings.AlgaePivotPID.setD(SmartDashboard.getNumber("Tune Algae Pivot kD", AlgaeArmSettings.AlgaePivotPID.getD()));

    }


    public void Enable() { enabled = true; }
    public void Disable() { enabled = false; }
    public void toggleEnabled() { enabled = !enabled; }


    public void presetIntakePosition() {
        TargetPivotAngle = AlgaeArmSettings.PresetPickupAngle;
        AlgaeRollerPower = AlgaeArmSettings.IntakePower;
    }

    public void presetStorePosition() {
        TargetPivotAngle = AlgaeArmSettings.PresetStoredAngle;
        AlgaeRollerPower = AlgaeArmSettings.HoldPower;
    }

    public void presetOuttakePosition() {
        TargetPivotAngle = AlgaeArmSettings.PresetOuttakeAngle;
        // AlgaeRollerPower = AlgaeArmSettings.OuttakePower;
    }

    public void stopRoller() { AlgaeRollerPower = 0; }
    public void intakeRoller() { AlgaeRollerPower = AlgaeArmSettings.IntakePower; }
    public void holdRoller() { AlgaeRollerPower = AlgaeArmSettings.HoldPower; }
    public void outtakeRoller() { AlgaeRollerPower = AlgaeArmSettings.OuttakePower; }


}
