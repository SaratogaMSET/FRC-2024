// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Intake.IntakeDefaultCommand;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystemIO;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystemIOSim;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystemIOTalon;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystem;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystemIO;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystemIOSim;
import frc.robot.Constants.IntakeSubsystem.Arm;
import frc.robot.Constants.IntakeSubsystem.Arm.Sim;

public class RobotContainer {
  public static ArmSubsystemIO armIO = Robot.isReal() ? new ArmSubsystemIOTalon() : new ArmSubsystemIOSim("Arm", null);
  public static ArmSubsystem arm = new ArmSubsystem(armIO);
  public static RollerSubsystemIO roller = Robot.isReal() ? new RollerSubsystem() : new RollerSubsystemIOSim();
  public final static CommandXboxController m_driverController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();

  }

  private void configureBindings() {
    // General control of wrist and shoulder angle using joysticks (for demonstration of sim)
    arm.setDefaultCommand(new IntakeDefaultCommand(arm, armIO, ()->(m_driverController.getLeftY()),  // Question for Govind: should we include elevator in this sim?
        ()->(m_driverController.getRightY()), Sim.SPEED));

    // Control of arm to a set angle using a button (I tested this using the wpilib simGUI which I can't find how to map xbox controller buttons, so will test when in room on Friday)
    m_driverController.a().whileTrue(new IntakeDefaultCommand(arm, armIO, ()->Arm.WRIST_HIGH_BOUND,
        ()->Arm.SHOULDER_HIGH_BOUND, Sim.SPEED));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
