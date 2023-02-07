package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class goHighCommand extends CommandBase {
    ElevatorSubsystem elevator;
    double high = 0;
    double range = 0;

    public goHighCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        elevator.goHigh(high, range);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setStop();
    }

    @Override
    public boolean isFinished() {
        if (elevator.inHighRange(high, range)) {
            return true;
        }
        return false;
    }

}
