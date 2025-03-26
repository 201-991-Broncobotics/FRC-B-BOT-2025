package frc.robot.subsystems;

import java.lang.annotation.Target;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Settings.CoralClawSettings;
import frc.robot.Settings.CoralSystemPresets;
import frc.robot.Settings.CoralSystemSettings;
import frc.robot.utility.CoralSystemPreset;
import frc.robot.utility.ElapsedTime;
import frc.robot.utility.Functions;
import frc.robot.utility.ElapsedTime.Resolution;
import frc.robot.Constants.MotorConstants;

public class CoralElevatorSystem extends SubsystemBase {
    public static int ElevatorStage;

    private double TargetElevatorHeight, ElevatorError;
    private double CurrentElevatorHeight;

    private ElevatorFeedforward elevatorFeedForward;

    private TalonFX elevator;

    private ElapsedTime runTime;
    private double frameTime = 0;

    private DoubleSupplier ManualControlAxis = () -> 0;
    private boolean GoToPosition = false; // whether or not the manual control controls the power up and down or sets the position

    private int numberOfStages = 5;

    private boolean overrideManualControl = false;
    private double lastManualControl = 0;
    private double overrideOverrideTolerance = 0.05; // units are joystick input

    private DiffyCoralClaw coralClawReference;


    //temp
    //private DoubleSupplier testEle;


    public CoralElevatorSystem(DiffyCoralClaw coralClawReference) {
        this.coralClawReference = coralClawReference;
        elevator = new TalonFX(MotorConstants.elevatorID);
        elevator.setNeutralMode(NeutralModeValue.Brake);
        TargetElevatorHeight = 0.0;
        elevatorFeedForward = new ElevatorFeedforward(CoralSystemSettings.kSE, CoralSystemSettings.kGE, CoralSystemSettings.kVE);

        runTime = new ElapsedTime(Resolution.SECONDS);

        //put numbers so we can grab latter brr
        SmartDashboard.putNumber("Ele kSE", CoralSystemSettings.kSA);
        SmartDashboard.putNumber("Ele kGE", CoralSystemSettings.kGA);
        SmartDashboard.putNumber("Ele kVE", CoralSystemSettings.kVA);
        SmartDashboard.putNumber("TargetAngle", 0);

        frameTime = runTime.time();
        runTime.reset();
    }
    
    public void setElevatorPos(double pos){
        TargetElevatorHeight = pos;
    }

    
    public void update() {
        //if(testEle!=null){ elevator.set(-testEle.getAsDouble()); }

        if (!GoToPosition) {

            if (coralClawReference.combinedElevatorCoralPitchControl(ManualControlAxis.getAsDouble(), 
                    (TargetElevatorHeight == CoralSystemSettings.minHeight) ? CoralSystemPresets.GroundIntake.clawPitch : Math.toRadians(20), // min angle is 0 unless elevator is at the bottom
                    (TargetElevatorHeight == CoralSystemSettings.maxHeight) ? CoralClawSettings.maxPitch : Math.toRadians(75) // max angle is 75 unless elevator is at the top
            )) { // ^ this returns true if the diffy arm is already at its min or max angle and the elevator can move instead
                if (Math.abs(ManualControlAxis.getAsDouble()) > overrideOverrideTolerance && overrideManualControl) overrideManualControl = false;
                if (!overrideManualControl) TargetElevatorHeight += ManualControlAxis.getAsDouble() * CoralSystemSettings.manualControlSpeed * frameTime;
            }

            //if (Math.abs(ManualControlAxis.getAsDouble()) > overrideOverrideTolerance && overrideManualControl) overrideManualControl = false;
            //if (!overrideManualControl) TargetElevatorHeight += ManualControlAxis.getAsDouble() * CoralSystemSettings.manualControlSpeed * frameTime;
        } else {
            if (Math.abs(ManualControlAxis.getAsDouble() - lastManualControl) > overrideOverrideTolerance && overrideManualControl) overrideManualControl = false;
            if (!overrideManualControl) TargetElevatorHeight = ManualControlAxis.getAsDouble() * (CoralSystemSettings.maxHeight - CoralSystemSettings.minHeight) + CoralSystemSettings.minHeight;
        }

        if (overrideManualControl) {
            switch (ElevatorStage) {
                case 0: goToPreset(CoralSystemPresets.GroundIntake); break;
                case 1: goToPreset(CoralSystemPresets.L1Reef); break;
                case 2: goToPreset(CoralSystemPresets.CoralStationIntake); break;
                case 3: goToPreset(CoralSystemPresets.L2Reef); break;
                case 4: goToPreset(CoralSystemPresets.L3Reef); break;
                case 5: goToPreset(CoralSystemPresets.L4Reef); break;
            }
        }
        
      
        //Update elevator position
        if (-elevator.getPosition().getValueAsDouble()<0) 
            CurrentElevatorHeight = 0;
        else CurrentElevatorHeight = -elevator.getPosition().getValueAsDouble()*CoralSystemSettings.elevatorRotationsToInches; //add offset later

        // limits
        TargetElevatorHeight = Functions.minMaxValue(CoralSystemSettings.minHeight, CoralSystemSettings.maxHeight, TargetElevatorHeight);

        //Calculate error
        ElevatorError=TargetElevatorHeight-CurrentElevatorHeight;
        
        //Move motor
        if(Math.abs(ElevatorError)<CoralSystemSettings.elevatorTolerance) {
            elevator.setVoltage(-elevatorFeedForward.calculate(0));
        } else{
            elevator.setVoltage(-elevatorFeedForward.calculate(ElevatorError/CoralSystemSettings.elevatorSpeedControl));
        }
        
    }


    @Override
    public void periodic() {
        frameTime = runTime.time();
        runTime.reset();
        
        //Smart Dashboard updates
        SmartDashboard.putNumber("ElevatorHeight", CurrentElevatorHeight);
        SmartDashboard.putNumber("TargetEle", TargetElevatorHeight);
        SmartDashboard.putNumber("PowertoElevator",  elevatorFeedForward.calculate(ElevatorError/CoralSystemSettings.elevatorSpeedControl));
        SmartDashboard.putString( "actual values ele", ""+elevatorFeedForward.getKs()+" "+elevatorFeedForward.getKg()+" "+elevatorFeedForward.getKv());
       // SmartDashboard.putString("test", testString);//hahahaahahaha I AM DEFINITLY OKAY RIGHT NOW
        SmartDashboard.putNumber("Left ElevatorAct ", elevator.getPosition().getValueAsDouble());

        //update ff
        /*boolean check1 = elevatorFeedForward.getKs()!=SmartDashboard.getNumber("Ele kSE", CoralSystemSettings.kSE);
        boolean check2 = elevatorFeedForward.getKg()!=SmartDashboard.getNumber("Ele kGE", CoralSystemSettings.kGE);
        boolean check3 = elevatorFeedForward.getKv()!=SmartDashboard.getNumber("Ele kVE", CoralSystemSettings.kVE);
        if(check1 || check2 || check3){
            SmartDashboard.putString( "stuff", "e");
            elevatorFeedForward = new ElevatorFeedforward(
            SmartDashboard.getNumber("Ele kSE", CoralSystemSettings.kSE), 
            SmartDashboard.getNumber("Ele kGE", CoralSystemSettings.kGE), 
            SmartDashboard.getNumber("Ele kVE", CoralSystemSettings.kVE));
        }*/
        
        
        
    }
    public boolean atPosition(){
        if(Math.abs(ElevatorError)<CoralSystemSettings.elevatorTolerance) return true;
        return false;
    }
    public int getStage(){
        return ElevatorStage;
    }
    public void setStage(int newStage){
        ElevatorStage = newStage;

        overrideManualControl = true;
        if (GoToPosition) lastManualControl = ManualControlAxis.getAsDouble();
    }

    public void upOneStage() {
        ElevatorStage += 1;
        if (ElevatorStage > numberOfStages) ElevatorStage = numberOfStages;

        overrideManualControl = true;
        if (GoToPosition) lastManualControl = ManualControlAxis.getAsDouble();
    }

    public void downOneStage() {
        ElevatorStage -= 1;
        if (ElevatorStage < 0) ElevatorStage = 0;

        overrideManualControl = true;
        if (GoToPosition) lastManualControl = ManualControlAxis.getAsDouble();
    }

    

    public void goToPreset(CoralSystemPreset coralSystemPreset) {
        setElevatorPos(coralSystemPreset.eleHeight);

        overrideManualControl = true;
        if (GoToPosition) lastManualControl = ManualControlAxis.getAsDouble();
    }

    public void setManualControl(DoubleSupplier controlAxis, boolean goToPosition) {
        ManualControlAxis = controlAxis;
        GoToPosition = goToPosition;
    }

    public void setManualControl(DoubleSupplier controlAxis) {
        setManualControl(controlAxis, false);
    }


}
