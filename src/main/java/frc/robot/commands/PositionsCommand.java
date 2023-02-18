package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class PositionsCommand extends CommandBase{
    ElevatorSubsystem elevator;
    double setSetPoint;
    double setpoint;
    String elevatorPosition;

     public PositionsCommand(ElevatorSubsystem elevatorSubsystem, double setpoint){
        elevator = elevatorSubsystem;
        this.setpoint = setpoint;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
        elevator.setSetpoint(setpoint);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    } 
}
