package frc.robot.commands;

import frc.robot.subsystems.CoralElevatorSystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralArmTeleOpCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final CoralElevatorSystem coralArmSystem;
    private final int direction;


    public CoralArmTeleOpCommand(CoralElevatorSystem subsystem, int direction) {
        this.direction =direction;
        coralArmSystem=subsystem;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(this.direction>0&coralArmSystem.getStage()<3) coralArmSystem.setStage(coralArmSystem.getStage()+1);
        if(this.direction<0&coralArmSystem.getStage()>0) coralArmSystem.setStage(coralArmSystem.getStage()-1);
      
        switch (coralArmSystem.getStage()) {//todo make settings
            case 0:
                coralArmSystem.setElevatorPos(0); // All of these are in Christian units 
                break;
            case 1:
                coralArmSystem.setElevatorPos(10);
                break;
            case 2:
                coralArmSystem.setElevatorPos(20);
                break;
            case 3:
                coralArmSystem.setElevatorPos(30);
                break;
            default:
                break;
        }
        SmartDashboard.putNumber("Elevator Stage", coralArmSystem.getStage());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
