// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Intake.IntakeDefaultCommand;
import frc.robot.commands.Intake.ManualRollersCommand;
import frc.robot.subsystems.IntakeSubsystem.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ActuatorShoulder.ActuatorShoulderIO;
import frc.robot.subsystems.IntakeSubsystem.ActuatorShoulder.ActuatorShoulderIOReal;
import frc.robot.subsystems.IntakeSubsystem.ActuatorShoulder.ActuatorShoulderIOSim;
import frc.robot.subsystems.IntakeSubsystem.ActuatorWrist.ActuatorWristIO;
import frc.robot.subsystems.IntakeSubsystem.ActuatorWrist.ActuatorWristIOReal;
import frc.robot.subsystems.IntakeSubsystem.ActuatorWrist.ActuatorWristIOSim;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystem;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystemIO;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystemIOSim;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystemIOTalon;
import frc.robot.Constants.Mode;
import frc.robot.Constants.Intake.AcutatorConstants.ActuatorState;
import frc.robot.Constants.Intake.Roller.RollerState;

public class RobotContainer {
  public static ActuatorShoulderIO actuatorShoulderIO = Robot.isReal() ? new ActuatorShoulderIOReal() : new ActuatorShoulderIOSim();
  public static ActuatorWristIO actuatorWristIO = Robot.isReal() ? new ActuatorWristIOReal() : new ActuatorWristIOSim();
  public static IntakeSubsystem intake = new IntakeSubsystem(actuatorShoulderIO, actuatorWristIO);
  public static RollerSubsystemIO rollerIO = Robot.isReal() ? new RollerSubsystemIOTalon() : new RollerSubsystemIOSim();
  public static RollerSubsystem roller = new RollerSubsystem(rollerIO);
  public final static CommandXboxController m_driverController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();

  }

  /* TODO:
   * Configure feedforward + PID trapezoid motion profile for shoulder joint
   * Write + test IR gate logic for counting in roller subsystem
   */
  private void configureBindings() {
    m_driverController.a().onTrue(new IntakeDefaultCommand(intake, ActuatorState.AMP));
    m_driverController.b().onTrue(new IntakeDefaultCommand(intake, ActuatorState.TRAP));
    m_driverController.x().onTrue(new IntakeDefaultCommand(intake, ActuatorState.SOURCE));
    m_driverController.y().onTrue(new IntakeDefaultCommand(intake, ActuatorState.NEUTRAL));
    m_driverController.leftBumper().onTrue(new IntakeDefaultCommand(intake, ActuatorState.GROUND_DEPLOY));

    m_driverController.rightBumper().toggleOnTrue(new ManualRollersCommand(roller, RollerState.INTAKE));
    m_driverController.rightBumper().toggleOnFalse(new ManualRollersCommand(roller, RollerState.OUTTAKE));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
