package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Settings.ClimbingSettings;
import frc.robot.utility.ElapsedTime;
import frc.robot.utility.Functions;
import frc.robot.utility.ElapsedTime.Resolution;
import frc.robot.utility.ThroughBoreEncoder;

public class ClimbingSystem extends SubsystemBase {

    private double climbingSpeed;
    //private SparkMax climbingMotor;
    private StatusCode climbingMotorConfig;
    private TalonFX climbingMotor;
    private CurrentLimitsConfigs currentLimitConfigs;

    private double ClimbingPower = 0;

    private DoubleSupplier climbEncoder;

    private ThroughBoreEncoder cThroughBore;

    private double climbingTargetPosition = 0;

    private ElapsedTime runTime;
    private double frameTime = 0;

    //private SparkMaxConfig climbMotorConfig;


    public ClimbingSystem() {

        runTime = new ElapsedTime(Resolution.SECONDS);

        //climbingMotor = new SparkMax(MotorConstants.climbingMotorID, MotorType.kBrushless);
        //climbMotorConfig = new SparkMaxConfig();

        //climbMotorConfig.idleMode(IdleMode.kBrake);

        //climbingMotor.configure(climbMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        climbingMotor = new TalonFX(MotorConstants.climbingMotorID);
        //climbingMotor.setControl(climbingMotorConfig);

        // current limit if needed
        currentLimitConfigs = new CurrentLimitsConfigs() 
            .withStatorCurrentLimit(Amps.of(ClimbingSettings.currentLimit))
            .withStatorCurrentLimitEnable(ClimbingSettings.useCurrentLimit);

        climbingMotorConfig = climbingMotor.getConfigurator().apply(currentLimitConfigs);

        climbingMotor.setNeutralMode(NeutralModeValue.Brake);
        

        climbingSpeed = ClimbingSettings.climbingSpeed;
        ClimbingPower = 0;

        cThroughBore = new ThroughBoreEncoder(6, 7, 8);
       
        cThroughBore.setAbsoluteZero(0);

        climbEncoder = () -> -cThroughBore.getAbsoluteAngleNorm(); // climbingMotor.getPosition().getValueAsDouble() * 1.0/9.0 * 1.0/5.0 * 1.0/5.0 * 2*Math.PI;

        frameTime = runTime.time();
        runTime.reset();

        SmartDashboard.putNumber("Tune Climbing kP", ClimbingSettings.climbPID.getP());
        SmartDashboard.putNumber("Tune Climbing maxPosition", ClimbingSettings.maxEncoderPosition);
        SmartDashboard.putNumber("Tune Climbing minPosition", ClimbingSettings.minEncoderPosition);
        SmartDashboard.putNumber("Tune Climbing speed", ClimbingSettings.climbingSpeed);
        SmartDashboard.putNumber("Tune Climbing current limit", ClimbingSettings.currentLimit);
        SmartDashboard.putBoolean("Tune Climbing use current limit", ClimbingSettings.useCurrentLimit);
    }

    public void update() {

        climbingTargetPosition += ClimbingPower * frameTime;

        climbingTargetPosition = Functions.minMaxValue(ClimbingSettings.minEncoderPosition, ClimbingSettings.maxEncoderPosition, climbingTargetPosition);

        double theClimbingPower = ClimbingSettings.climbPID.calculate(climbEncoder.getAsDouble(), climbingTargetPosition);

        if (climbEncoder.getAsDouble() > ClimbingSettings.maxEncoderPosition && theClimbingPower > 0) theClimbingPower = 0;
        if (climbEncoder.getAsDouble() < ClimbingSettings.minEncoderPosition && theClimbingPower < 0) theClimbingPower = 0;
        climbingMotor.set(-theClimbingPower);
    }


    @Override
    public void periodic() {
        frameTime = runTime.time();
        runTime.reset();

        SmartDashboard.putNumber("Climb Motor Current", climbingMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Climb Encoder Raw", climbEncoder.getAsDouble());
        SmartDashboard.putNumber("Climb Encoder", Math.toDegrees(climbEncoder.getAsDouble()));
        SmartDashboard.putNumber("Climb Target Position", Math.toDegrees(climbingTargetPosition));

        climbingSpeed = ClimbingSettings.climbingSpeed;


        ClimbingSettings.climbPID.setP(SmartDashboard.getNumber("Tune Climbing kP", ClimbingSettings.climbPID.getP()));
        ClimbingSettings.maxEncoderPosition = SmartDashboard.getNumber("Tune Climbing maxPosition", ClimbingSettings.maxEncoderPosition);
        ClimbingSettings.minEncoderPosition = SmartDashboard.getNumber("Tune Climbing minPosition", ClimbingSettings.minEncoderPosition);
        ClimbingSettings.climbingSpeed = SmartDashboard.getNumber("Tune Climbing speed", ClimbingSettings.climbingSpeed);
        ClimbingSettings.currentLimit = (int) SmartDashboard.getNumber("Tune Climbing current limit", ClimbingSettings.currentLimit);
        ClimbingSettings.useCurrentLimit = SmartDashboard.getBoolean("Tune Climbing use current limit", ClimbingSettings.useCurrentLimit);

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void StartClimbing() {
        ClimbingPower = -climbingSpeed;
    }

    public void StartUnclimbing() {
        ClimbingPower = climbingSpeed;
    }

    public void StopClimbing() {
        ClimbingPower = 0;
    }

}