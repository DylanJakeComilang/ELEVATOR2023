package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class resetEnCommand extends CommandBase{
    ElevatorSubsystem elevator;
    double speed;

    public resetEnCommand(ElevatorSubsystem elevatorSubsystem){
        elevator = elevatorSubsystem;
        addRequirements(elevator);
    }

    
    
    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
        elevator.resetEncoder();
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
