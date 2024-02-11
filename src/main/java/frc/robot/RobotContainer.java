// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Mode;
import frc.robot.commands.Drivetrain.FeedForwardCharacterization;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Swerve.GyroIO;
import frc.robot.subsystems.Swerve.GyroIOPigeon2;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
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
  private final CommandXboxController controller = new CommandXboxController(0);
  // private final VisionSubsystem m_visionSubsystem = new VisionSubsystem( Constants.currentMode == Mode.REAL ? new VisionIOReal() : new VisionIOSim());
  private final SwerveSubsystem swerve =
        new SwerveSubsystem(
            Constants.currentMode == Mode.REAL
                ? SwerveSubsystem.createCamerasReal()
                : Constants.currentMode == Mode.SIM ? SwerveSubsystem.createCamerasSim()
                : SwerveSubsystem.createVisionIOs(),
            Constants.currentMode == Mode.REAL ? new GyroIOPigeon2() : new GyroIO() {},
            Constants.currentMode == Mode.REAL
                ? SwerveSubsystem.createTalonFXModules()
                : Constants.currentMode == Mode.SIM ? SwerveSubsystem.createSimModules()
                : SwerveSubsystem.createModuleIOs());

  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

  public static ActuatorShoulderIO actuatorShoulderIO = Robot.isReal() ? new ActuatorShoulderIOReal() : new ActuatorShoulderIOSim();
  public static ActuatorWristIO actuatorWristIO = Robot.isReal() ? new ActuatorWristIOReal() : new ActuatorWristIOSim();
  public static IntakeSubsystem intake = new IntakeSubsystem(actuatorShoulderIO, actuatorWristIO);
  public static RollerSubsystemIO rollerIO = Robot.isReal() ? new RollerSubsystemIOTalon() : new RollerSubsystemIOSim();
  public static RollerSubsystem roller = new RollerSubsystem(rollerIO);
  public final static CommandXboxController m_driverController = new CommandXboxController(0);

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
    autoChooser.addOption(
        "PID Rotation Auton", new PathPlannerAuto("PID Rotation Auton"));
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
                      -rotationInput(controller.getLeftTriggerAxis()) * SwerveSubsystem.MAX_ANGULAR_SPEED)));
    }
    else{
      swerve.setDefaultCommand(
            swerve.runVelocityFieldRelative(
                () ->
                    new ChassisSpeeds(
                        -translationInput(controller.getLeftY()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                        -translationInput(controller.getLeftX()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                        -rotationInput(controller.getRightX()) * SwerveSubsystem.MAX_ANGULAR_SPEED)));
    }
     controller.y().onTrue(Commands.runOnce(() -> swerve.setYaw(Rotation2d.fromDegrees(0))));

    // intake.setDefaultCommand(new IntakeDefaultCommand(intake,ActuatorState.NEUTRAL));
    m_driverController.a().whileTrue((new IntakeDefaultCommand(intake, ActuatorState.AMP))).onFalse(
      new IntakeDefaultCommand(intake, ActuatorState.NEUTRAL)
    );
    m_driverController.b().whileTrue(new IntakeDefaultCommand(intake, ActuatorState.TRAP)).onFalse(
      new IntakeDefaultCommand(intake, ActuatorState.NEUTRAL)
    );
    m_driverController.x().whileTrue(new IntakeDefaultCommand(intake, ActuatorState.SOURCE)).onFalse(
      new IntakeDefaultCommand(intake, ActuatorState.NEUTRAL)
    );
    m_driverController.rightBumper().toggleOnTrue(new ManualRollersCommand(roller, RollerState.INTAKE));
    m_driverController.rightBumper().toggleOnFalse(new ManualRollersCommand(roller, RollerState.OUTTAKE));
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
