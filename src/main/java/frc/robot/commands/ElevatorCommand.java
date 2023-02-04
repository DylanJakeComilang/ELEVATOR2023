package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

public class ElevatorCommand extends CommandBase{
    ElevatorSubsystem elevator;
    DoubleSupplier doubleSupplier;
    double speed;

    public ElevatorCommand(ElevatorSubsystem elevatorSubsystem, DoubleSupplier doubleSupplier){
        elevator = elevatorSubsystem;
        this.doubleSupplier = doubleSupplier;
        addRequirements(elevator);
    }

    
    
    @Override
    public void initialize() {
        elevator.initialize();
    }
 
    public void execute() {
        double joystickSpeed = doubleSupplier.getAsDouble();
        elevator.manualElevator(joystickSpeed);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
