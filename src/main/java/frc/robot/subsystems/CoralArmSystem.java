package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings.CoralSystemSettings;
import frc.robot.Constants.MotorConstants;

public class CoralArmSystem extends SubsystemBase {
    private int ElevatorStage;

    private double TargetElevatorHeight, ElevatorError;
    private double CurrentElevatorHeight;

    private String testString;

    private ElevatorFeedforward elevatorFeedForward;

    private TalonFX elevator;

    //temp
    private DoubleSupplier testEle;


    public CoralArmSystem(String s) {
        elevator = new TalonFX(MotorConstants.elevatorID);
        elevator.setNeutralMode(NeutralModeValue.Brake);
        TargetElevatorHeight = 0.0;
        elevatorFeedForward = new ElevatorFeedforward(CoralSystemSettings.kSE, CoralSystemSettings.kGE, CoralSystemSettings.kVE);

        testString = s;

        //put numbers so we can grab latter brr
        SmartDashboard.putNumber("Ele kSE", CoralSystemSettings.kSA);
        SmartDashboard.putNumber("Ele kGE", CoralSystemSettings.kGA);
        SmartDashboard.putNumber("Ele kVE", CoralSystemSettings.kVA);
        SmartDashboard.putNumber("TargetAngle", 0);
    }
    public CoralArmSystem(DoubleSupplier eleControl) {
        //leftElevator = new TalonFX(MotorConstants.coralLeftElevatorID);
        //rightElevator = new TalonFX(MotorConstants.coralRightElevatorID);
        //coralPivot = new TalonFX(MotorConstants.coralPivotID);
        testEle=eleControl;

        elevatorFeedForward = new ElevatorFeedforward(CoralSystemSettings.kSE, CoralSystemSettings.kGE, CoralSystemSettings.kVE);
    }
    public void setElevatorPos(double pos){
        TargetElevatorHeight = pos;
    }

    
    public void update() {
        
        if(testEle!=null){
            elevator.set(-testEle.getAsDouble());
        }
      
        //Update elevator position
        if (-elevator.getPosition().getValueAsDouble()<0) 
            CurrentElevatorHeight = 0;
        else CurrentElevatorHeight = -elevator.getPosition().getValueAsDouble()*CoralSystemSettings.elevatorRotationsToInches; //add offset later


        //Calculate error
        ElevatorError=TargetElevatorHeight-CurrentElevatorHeight;
        
        //Move motors
        if(Math.abs(ElevatorError)<CoralSystemSettings.elevatorTolerance) {
            elevator.setVoltage(-elevatorFeedForward.calculate(0));
        } else{
            elevator.setVoltage(-elevatorFeedForward.calculate(ElevatorError/CoralSystemSettings.elevatorSpeedControl));
        }
        
        

    }


    @Override
    public void periodic() {


        
        //Smart Dashboard updates
        SmartDashboard.putNumber("ElevatorHeight", CurrentElevatorHeight);
        SmartDashboard.putNumber("TargetEle", TargetElevatorHeight);
        SmartDashboard.putNumber("PowertoElevator",  elevatorFeedForward.calculate(ElevatorError/CoralSystemSettings.elevatorSpeedControl));
        SmartDashboard.putString( "actual values ele", ""+elevatorFeedForward.getKs()+" "+elevatorFeedForward.getKg()+" "+elevatorFeedForward.getKv());
       // SmartDashboard.putString("test", testString);//hahahaahahaha I AM DEFINITLY OKAY RIGHT NOW
        SmartDashboard.putNumber("Left ElevatorAct ", elevator.getPosition().getValueAsDouble());

        //update ff
        boolean check1 = elevatorFeedForward.getKs()!=SmartDashboard.getNumber("Ele kSE", CoralSystemSettings.kSE);
        boolean check2 = elevatorFeedForward.getKg()!=SmartDashboard.getNumber("Ele kGE", CoralSystemSettings.kGE);
        boolean check3 = elevatorFeedForward.getKv()!=SmartDashboard.getNumber("Ele kVE", CoralSystemSettings.kVE);
        if(check1 || check2 || check3){
            SmartDashboard.putString( "stuff", "e");
            elevatorFeedForward = new ElevatorFeedforward(
            SmartDashboard.getNumber("Ele kSE", CoralSystemSettings.kSE), 
            SmartDashboard.getNumber("Ele kGE", CoralSystemSettings.kGE), 
            SmartDashboard.getNumber("Ele kVE", CoralSystemSettings.kVE));
        }
        
        
        
    }
    public boolean atPosition(){
        if(Math.abs(ElevatorError)<CoralSystemSettings.elevatorTolerance)
        return true;
        return false;
    }
    public int getStage(){
        return ElevatorStage;
    }
    public void setStage(int newStage){
        ElevatorStage = newStage;
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }


}
