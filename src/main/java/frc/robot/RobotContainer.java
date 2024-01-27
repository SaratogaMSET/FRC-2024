// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.Drivetrain.FeedForwardCharacterization;
import frc.robot.subsystems.Swerve.GyroIO;
import frc.robot.subsystems.Swerve.GyroIOPigeon2;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.Mode;

public class RobotContainer {
  SendableChooser autoChooser = new SendableChooser<>();
  private final CommandXboxController controller = new CommandXboxController(0);
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final SwerveSubsystem swerve =
        new SwerveSubsystem(
          m_visionSubsystem::getPose2d,
          m_visionSubsystem::getTimestamp,
          m_visionSubsystem::getScaledSTDDevs,
            Constants.currentMode == Mode.REAL ? new GyroIOPigeon2() : new GyroIO() {},
            Constants.currentMode == Mode.REAL
                ? SwerveSubsystem.createTalonFXModules()
                : Constants.currentMode == Mode.SIM ? SwerveSubsystem.createSimModules()
                : SwerveSubsystem.createModuleIOs());
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("Feedforward Characterization", new FeedForwardCharacterization(swerve, swerve::runCharacterizationVoltsCmd, swerve::getCharacterizationVelocity));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData("Auto Chooser", autoChooser);
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
     controller.y().onTrue(Commands.runOnce(() -> swerve.setYaw(Rotation2d.fromDegrees(0))));
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Example Auto");
  }
}
