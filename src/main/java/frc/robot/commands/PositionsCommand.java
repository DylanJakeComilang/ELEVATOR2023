package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class PositionsCommand extends CommandBase{
    ElevatorSubsystem elevator;
    double setSetPoint;
    String elevatorPosition;

     public PositionsCommand(ElevatorSubsystem elevatorSubsystem , double setpoint){
        elevator = elevatorSubsystem;
        setSetPoint = setpoint;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {
        elevator.outputMotor(setSetPoint);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    } 
}
