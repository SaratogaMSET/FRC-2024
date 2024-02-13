// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class RobotContainer {
   public final static CommandXboxController m_driverController = new CommandXboxController(0);
   public static ElevatorIO elevatorIO = Robot.isReal() ? new ElevatorIOTalonFX() : new ElevatorIOSim();
   public static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(elevatorIO);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.a().toggleOnTrue((new RunCommand(()->elevatorSubsystem.setSetpoint(0.8))));
    m_driverController.a().toggleOnFalse((new RunCommand(()->elevatorSubsystem.setSetpoint(0.0))));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
