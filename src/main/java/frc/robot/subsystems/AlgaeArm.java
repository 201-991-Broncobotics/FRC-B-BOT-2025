package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.MotorConstants;

public class AlgaeArm extends SubsystemBase {
    
    private TalonFX pivotMotor;
    private SparkFlex algaeRoller;
    private SparkFlexConfig algaeRollerConfig;

    private DoubleSupplier algaePivotEncoder;


    public AlgaeArm(double StartingAngle) {
        
        algaeRollerConfig = new SparkFlexConfig();


        pivotMotor = new TalonFX(MotorConstants.algaePivotID);
        algaeRoller = new SparkFlex(MotorConstants.algaeRollerID, MotorType.kBrushless);
        algaeRollerConfig.idleMode(IdleMode.kCoast);
        algaeRoller.configure(algaeRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        algaePivotEncoder = () -> pivotMotor.getPosition().getValueAsDouble();

    }


    public void update() {

        

    }

    @Override
    public void periodic() { 

    }

}
