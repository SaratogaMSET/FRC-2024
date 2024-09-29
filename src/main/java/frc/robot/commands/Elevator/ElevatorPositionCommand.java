package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class ElevatorPositionCommand extends Command {

  ElevatorSubsystem elevator;
  double position;

  public ElevatorPositionCommand(ElevatorSubsystem elevatorSubsystem, double position) {
    this.elevator = elevatorSubsystem;
    this.position = position;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevator.setSetpoint(position);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setVoltage(0.0, 0.0);
  }
}
