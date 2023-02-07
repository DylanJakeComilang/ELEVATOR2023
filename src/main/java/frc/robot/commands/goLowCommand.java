package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class goLowCommand extends CommandBase {
    ElevatorSubsystem elevator;
    double low = 0;
    double range = 0;

    public goLowCommand(ElevatorSubsystem elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        elevator.goLow(low, range);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setStop();
    }

    @Override
    public boolean isFinished() {
        if (elevator.inLowRange(low, range)){
            return true;
        }
        return false;
    }
    
}
