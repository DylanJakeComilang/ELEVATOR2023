package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class goMiddleCommand extends CommandBase {
    ElevatorSubsystem elevator;
    double mid = 10000;
    double range = 100;

    public goMiddleCommand(ElevatorSubsystem elevator){
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        elevator.goMid(mid, range);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setStop();
    }

    @Override
    public boolean isFinished() {
        if (elevator.inMidRange(mid, range)){
            return true;
        }
        return false;
    }
    
}
