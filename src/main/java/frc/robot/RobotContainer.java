// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Intake.IntakeDefaultCommand;
import frc.robot.commands.Intake.ManualRollersCommand;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.WristSubsystemIO;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.WristSubsystemIOSim;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.WristSubsystemIOTalonFX;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ShoulderSubsystem.ShoulderSubsystemIO;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ShoulderSubsystem.ShoulderSubsystemIOTalonFX;
import frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.ShoulderSubsystem.ShoulderSubsystemIOSim;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystemIOTalon;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystem;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystemIO;
import frc.robot.subsystems.IntakeSubsystem.RollerSubsystem.RollerSubsystemIOSim;
import frc.robot.Constants.IntakeSubsystem.Arm.ArmState;
import frc.robot.Constants.IntakeSubsystem.Roller.RollerState;

public class RobotContainer {

  public static ShoulderSubsystemIO shoulderIO = Robot.isReal() ? new ShoulderSubsystemIOTalonFX() : new ShoulderSubsystemIOSim();
  public static WristSubsystemIO wristIO = Robot.isReal() ? new WristSubsystemIOTalonFX() : new WristSubsystemIOSim(null, null);
  
  public static ArmSubsystem arm = new ArmSubsystem(shoulderIO, wristIO);
  // public static RollerSubsystemIO rollerIO = Robot.isReal() ? new RollerSubsystemIOTalon() : new RollerSubsystemIOSim();
  // public static RollerSubsystem roller = new RollerSubsystem(rollerIO);
  public final static CommandXboxController m_driverController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();

  }

  private void configureBindings() {
    m_driverController.a().onTrue(new IntakeDefaultCommand(arm, ArmState.AMP));
    m_driverController.b().onTrue(new IntakeDefaultCommand(arm, ArmState.TRAP));
    m_driverController.x().onTrue(new IntakeDefaultCommand(arm, ArmState.SOURCE));
    m_driverController.y().onTrue(new IntakeDefaultCommand(arm, ArmState.NEUTRAL));
    m_driverController.leftBumper().onTrue(new IntakeDefaultCommand(arm, ArmState.GROUND_DEPLOY));  // FIXME: This incorrectly runs the motor at like full speed (the issue is the shoulderSetAngle() function)

    // m_driverController.rightBumper().toggleOnTrue(new ManualRollersCommand(roller, RollerState.AMP_INTAKE));
    // m_driverController.rightBumper().toggleOnFalse(new ManualRollersCommand(roller, RollerState.OUTTAKE));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
