// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Swerve.GyroIO;
import frc.robot.subsystems.Swerve.GyroIOPigeon2;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(0);
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final SwerveSubsystem swerve =
        new SwerveSubsystem(
          m_visionSubsystem::getPose2d,
          m_visionSubsystem::getTimestamp,
          m_visionSubsystem::getScaledSTDDevs,
           Robot.isReal() ? new GyroIOPigeon2() : new GyroIO() {},
              Robot.isReal()
                ? SwerveSubsystem.createTalonFXModules()
                : Constants.currentMode == Constants.Mode.SIM ? SwerveSubsystem.createSimModules()
                : SwerveSubsystem.createModuleIOs());
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
  public RobotContainer() {
    // autoChooser = AutoBuilder.buildAutoChooser();
    // autoChooser.addOption("Feedforward Characterization", new FeedForwardCharacterization(swerve, swerve::runCharacterizationVoltsCmd, swerve::getCharacterizationVelocity));
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
    autoChooser.addOption(
        "PID Translation Auton", new PathPlannerAuto("PID Translation Auton"));
    configureBindings();
  }

  private void configureBindings() {
    if(Robot.isSimulation()){
      swerve.setDefaultCommand(
          swerve.runVelocityFieldRelative(
              () ->
                  new ChassisSpeeds(
                      -translationInput(controller.getLeftY()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                      -translationInput(controller.getLeftX()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                      rotationInput(controller.getLeftTriggerAxis()) * SwerveSubsystem.MAX_ANGULAR_SPEED)));
    }
    else{
      swerve.setDefaultCommand(
            swerve.runVelocityFieldRelative(
                () ->
                    new ChassisSpeeds(
                        -translationInput(controller.getLeftY()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                        -translationInput(controller.getLeftX()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                        rotationInput(controller.getRightX()) * SwerveSubsystem.MAX_ANGULAR_SPEED)));
    }
     controller.y().onTrue(Commands.runOnce(() -> swerve.setYaw(Rotation2d.fromDegrees(0))));
  }

  public double translationInput(double input){
    if(Math.abs(input) < 0.03) return 0;
    input = Math.signum(input) * input * input;
    if(Math.abs(input) > 1) input = Math.signum(input);
    return input;
  }
  public double rotationInput(double input){
    if(Math.abs(input) < 0.03) return 0;
    input = Math.signum(input) * input * input;
    if(Math.abs(input) > 1) input = Math.signum(input);
    return input;
  }
  public Command getAutonomousCommand() {
    // return swerve.runVelocityCmd(()->new ChassisSpeeds(1,0,0)).withTimeout(0.5);
    return autoChooser.get();
  }
}
