// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Swerve.GyroIO;
import frc.robot.Subsystems.Swerve.GyroIOPigeon2;
import frc.robot.Subsystems.Swerve.SwerveSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(0);
  private final SwerveSubsystem swerve =
        new SwerveSubsystem(
            Constants.currentMode == Mode.REAL ? new GyroIOPigeon2() : new GyroIO() {},
            Constants.currentMode == Mode.REAL
                ? SwerveSubsystem.createTalonFXModules()
                : SwerveSubsystem.createSimModules());
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    swerve.setDefaultCommand(
        swerve.runVelocityFieldRelative(
            () ->
                new ChassisSpeeds(
                    -controller.getLeftY() * SwerveSubsystem.MAX_LINEAR_SPEED,
                    -controller.getLeftX() * SwerveSubsystem.MAX_LINEAR_SPEED,
                    controller.getRightX() * SwerveSubsystem.MAX_ANGULAR_SPEED)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
