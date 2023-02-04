package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class PositionsCommand extends CommandBase{
   /* ElevatorSubsystem elevator;
    double setpoint;
    String elevatorPosition;

     public PositionsCommand(ElevatorSubsystem elevatorSubsystem, double setpoint, String elevatorPosition){
        elevator = elevatorSubsystem;
        this.elevatorPosition = elevatorPosition;
        this.setpoint = setpoint;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.initialize();
    }
    
    @Override
    public void execute() {
        SmartDashboard.putString("Elevator Position", elevatorPosition); // Prints the desired position
        if (elevator.getUpperLimit()){ // limit pressed -> stop
            elevator.setStop();
         } else if (elevator.getLowerLimit()){
             elevator.setStop();
             elevator.resetEncoder();
         }
        elevator.outputMotor(setpoint);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    } */
}
