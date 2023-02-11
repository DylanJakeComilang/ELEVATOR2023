// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.




package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.resetEnCommand;
import frc.robot.commands.PositionsCommand;
import frc.robot.commands.goHighCommand;
import frc.robot.commands.goLowCommand;
import frc.robot.commands.goMiddleCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final XboxController xbox = new XboxController(0);
  private final Joystick joystick = new Joystick(1);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
  new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command..CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command..CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command..CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
   // new JoystickButton(Joystick, 1).onTrue(new PositionsCommand(elevatorSubsystem,0,"Hybrid"));   // sets hybrid states
   // new JoystickButton(Joystick, 2).onTrue(new PositionsCommand(elevatorSubsystem,10000,"Middle")); // sets middle states
   // new JoystickButton(Joystick, 3).onTrue(new PositionsCommand(elevatorSubsystem,20000,"High")); // sets high states
  

   // new JoystickButton(joystick, 2).onTrue(new goLowCommand(elevatorSubsystem));
   // new JoystickButton(joystick, 3).onTrue(new goMiddleCommand(elevatorSubsystem));
   // new JoystickButton(joystick, 4).onTrue(new goHighCommand(elevatorSubsystem));

    new JoystickButton(joystick,11).onTrue(new resetEnCommand(elevatorSubsystem)); // reset encoder

    /////////////////////////////////   | Once the button is held, motor can move manualy |
    //         Lock Position       //   |      after being let go, position is locked     |
    /////////////////////////////////   ↓                                                 ↓

    new JoystickButton(joystick, 5).whileTrue(new ElevatorCommand(elevatorSubsystem, () -> joystick.getY()));
    new JoystickButton(joystick, 2).onTrue(new PositionsCommand(elevatorSubsystem, 10));
  }


}
