package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorCommand extends CommandBase{
    ElevatorSubsystem elevator;
    double speed;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, double speed){
        elevator = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevator);
    }

    
    
    @Override
    public void initialize() {

    }
    
    @Override
    public void execute() {
    if (elevator.getUpperLimit()){
       elevator.setStop();
    } else if (elevator.getLowerLimit()){
        elevator.setStop();
        elevator.resetEncoder();
    } else{
        elevator.setSpeed(speed);
    }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
