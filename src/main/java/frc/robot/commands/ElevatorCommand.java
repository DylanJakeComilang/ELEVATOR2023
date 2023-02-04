package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

public class ElevatorCommand extends CommandBase{
    ElevatorSubsystem elevator;
    DoubleSupplier speed;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, DoubleSupplier speed){
        elevator = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevator);
    }

    
    
    @Override
    public void initialize() {

    }
 
    public void execute() {
     double elevatorSpeed = speed.getAsDouble();
    if (elevator.getUpperLimit()){
       elevator.setStop();
    } else if (elevator.getLowerLimit()){
        elevator.setStop();
        elevator.resetEncoder();
    } else{
        elevator.setSpeed(elevatorSpeed);
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