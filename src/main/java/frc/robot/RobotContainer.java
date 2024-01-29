// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Intake.ManualRollersCommand;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystemIO;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystemIOSim;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystemIOTalon;
import frc.robot.commands.Intake.ManualShoulder;
import frc.robot.commands.Intake.ManualWrist;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystem;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystemIO;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystemIOSim;

public class RobotContainer {
  public static ArmSubsystemIO arm = Robot.isReal() ? new ArmSubsystemIOTalon() : new ArmSubsystemIOSim("/Arm", null);
  public static RollerSubsystemIO roller = Robot.isReal() ? new RollerSubsystem() : new RollerSubsystemIOSim();
  public final static CommandXboxController m_driverController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_driverController.x().onTrue(new ManualRollersCommand(roller, 0.1));

    m_driverController.a().whileTrue(new ParallelCommandGroup(new ManualWrist(arm, 0.1, m_driverController.getLeftY()), new ManualShoulder(arm, 0.1, m_driverController.getRightY())));
    m_driverController.a().whileFalse(new ParallelCommandGroup(new ManualWrist(arm, 0.1, m_driverController.getLeftY()), new ManualShoulder(arm, 0.1, m_driverController.getRightY())));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
