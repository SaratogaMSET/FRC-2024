// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
package frc.robot;

import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.Shoulder;
import frc.robot.Constants.Intake.DesiredStates.ArmStates;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.commands.Intake.IntakeDefaultCommand;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.ActuatorShoulder.ActuatorShoulderIO;
import frc.robot.subsystems.Intake.ActuatorShoulder.ActuatorShoulderIOReal;
import frc.robot.subsystems.Intake.ActuatorShoulder.ActuatorShoulderIOSim;
import frc.robot.subsystems.Intake.ActuatorWrist.ActuatorWristIO;
import frc.robot.subsystems.Intake.ActuatorWrist.ActuatorWristIOReal;
import frc.robot.subsystems.Intake.ActuatorWrist.ActuatorWristIOSim;
import frc.robot.subsystems.Intake.RollerSubsystem.RollerSubsystem;
import frc.robot.subsystems.Intake.RollerSubsystem.RollerSubsystemIO;
import frc.robot.subsystems.Intake.RollerSubsystem.RollerSubsystemIOSim;
import frc.robot.subsystems.Intake.RollerSubsystem.RollerSubsystemIOTalon;
import frc.robot.subsystems.Swerve.GyroIO;
import frc.robot.subsystems.Swerve.GyroIOPigeon2;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.util.AllianceFlipUtil;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(0);
  // private final VisionSubsystem m_visionSubsystem = new VisionSubsystem( Robot.isReal ? new VisionIOReal() : new VisionIOSim());
  private SwerveSubsystem swerve = null;
  //  =
        // new SwerveSubsystem(
        //     Robot.isReal()
        //         ? SwerveSubsystem.createCamerasReal()
        //         : Constants.currentMode == Mode.SIM ? SwerveSubsystem.createCamerasSim()
        //         : SwerveSubsystem.createVisionIOs(),
        //     Robot.isReal() ? new GyroIOPigeon2() : new GyroIO() {},
        //     Robot.isReal()
        //         ? SwerveSubsystem.createTalonFXModules()
        //         : Constants.currentMode == Mode.SIM ? SwerveSubsystem.createSimModules()
        //         : SwerveSubsystem.createModuleIOs());

  private final LoggedDashboardChooser<Command> autoChooser; // = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
  private final LoggedDashboardChooser<RobotType> robotChooser = new LoggedDashboardChooser<>("Robot Choices", buildRobotChooser());

  public static ActuatorShoulderIO actuatorShoulderIO = null;// = Robot.isReal() ? new ActuatorShoulderIOReal() : new ActuatorShoulderIOSim();
  public static ActuatorWristIO actuatorWristIO = null; // = Robot.isReal() ? new ActuatorWristIOReal() : new ActuatorWristIOSim();
  public static IntakeSubsystem intake = null; // = new IntakeSubsystem(actuatorShoulderIO, actuatorWristIO);
  public static RollerSubsystemIO rollerIO = null; // = Robot.isReal() ? new RollerSubsystemIOTalon() : new RollerSubsystemIOSim();
  public static RollerSubsystem roller = null;//  = new RollerSubsystem(rollerIO);
  public static ElevatorIO elevatorIO = null;//  = Robot.isReal() ? new ElevatorIOTalonFX() : new ElevatorIOSim();
  public static ElevatorSubsystem elevator = null;// = new ElevatorSubsystem(elevatorIO);
  public final static CommandXboxController m_driverController = new CommandXboxController(0);

  public static SuperStructureVisualizer viz = new SuperStructureVisualizer(
    "SuperStructure", null, ()-> elevator.getSecondStageLength() ,()->elevator.getAverageExtension(), 
    ()->intake.shoulderGetDegrees(), () -> intake.wristGetDegrees());

  // public static TestSuperStructureVisualizer viz = new TestSuperStructureVisualizer("SuperStructure", null, ()->0.0, ()->0.0, ()->0.0, ()->0.0);

  public RobotContainer() {
    // autoChooser = AutoBuilder.buildAutoChooser();
    //  robotChooser = new LoggedDashboardChooser<>("Robot Choices", buildRobotChooser());

    // Constants.setRobot(robotChooser.get()); TODO: THE CHOOSER DOES NOT WORK.
    // assert Constants.getRobot() != null: "ROBOT IS NULL";
    // autoChooser.addOption("Feedforward Characterization", new FeedForwardCharacterization(swerve, swerve::runCharacterizationVoltsCmd, swerve::getCharacterizationVelocity));

  // Instantiate active subsystems
    if (Constants.getMode() != Mode.REPLAY && Robot.isReal()) {
      switch (Constants.getRobot()) {
        case ROBOT_2024C:
          swerve = new SwerveSubsystem(
              Robot.isReal()
                  ? SwerveSubsystem.createCamerasReal()
                  : SwerveSubsystem.createCamerasSim(),
              Robot.isReal() ? new GyroIOPigeon2() : new GyroIO() {},
              Robot.isReal()
                  ? SwerveSubsystem.createTalonFXModules()
                  : SwerveSubsystem.createSimModules());
          // actuatorShoulderIO = Robot.isReal() ? new ActuatorShoulderIOReal() : new ActuatorShoulderIOSim();
          // actuatorWristIO = Robot.isReal() ? new ActuatorWristIOReal() : new ActuatorWristIOSim();
          // intake = new IntakeSubsystem(actuatorShoulderIO, actuatorWristIO);
          // rollerIO = Robot.isReal() ? new RollerSubsystemIOTalon() : new RollerSubsystemIOSim();
          // roller = new RollerSubsystem(rollerIO);
          // elevatorIO = Robot.isReal() ? new ElevatorIOTalonFX() : new ElevatorIOSim();
          // elevator = new ElevatorSubsystem(elevatorIO);     
          break;
        case ROBOT_2024P:
            swerve = new SwerveSubsystem(
              Robot.isReal()
                  ? SwerveSubsystem.createCamerasReal()
                  : SwerveSubsystem.createCamerasSim(),
              Robot.isReal() ? new GyroIOPigeon2() : new GyroIO() {},
              Robot.isReal()
                  ? SwerveSubsystem.createTalonFXModules()
                  : SwerveSubsystem.createSimModules());
          break;
        case ROBOT_SIMBOT:
            swerve = new SwerveSubsystem(
              Robot.isReal()
                  ? SwerveSubsystem.createCamerasReal()
                  : SwerveSubsystem.createCamerasSim(),
              Robot.isReal() ? new GyroIOPigeon2() : new GyroIO() {},
              Robot.isReal()
                  ? SwerveSubsystem.createTalonFXModules()
                  : SwerveSubsystem.createSimModules());
          actuatorShoulderIO = Robot.isReal() ? new ActuatorShoulderIOReal() : new ActuatorShoulderIOSim();
          actuatorWristIO = Robot.isReal() ? new ActuatorWristIOReal() : new ActuatorWristIOSim();
          intake = new IntakeSubsystem(actuatorShoulderIO, actuatorWristIO);
          rollerIO = Robot.isReal() ? new RollerSubsystemIOTalon() : new RollerSubsystemIOSim();
          roller = new RollerSubsystem(rollerIO);
          elevatorIO = Robot.isReal() ? new ElevatorIOTalonFX() : new ElevatorIOSim();
          elevator = new ElevatorSubsystem(elevatorIO);     
          break;

        default:
          swerve = new SwerveSubsystem(
              Robot.isReal()
                  ? SwerveSubsystem.createCamerasReal()
                  : SwerveSubsystem.createCamerasSim(),
              Robot.isReal() ? new GyroIOPigeon2() : new GyroIO() {},
              Robot.isReal()
                  ? SwerveSubsystem.createTalonFXModules()
                  : SwerveSubsystem.createSimModules());
      }
    } else {
      // Instantiate as sim, if robot is not real
          swerve = new SwerveSubsystem(
              Robot.isReal()
                  ? SwerveSubsystem.createCamerasReal()
                  : SwerveSubsystem.createCamerasSim(),
              Robot.isReal() ? new GyroIOPigeon2() : new GyroIO() {},
              Robot.isReal()
                  ? SwerveSubsystem.createTalonFXModules()
                  : SwerveSubsystem.createSimModules());
          actuatorShoulderIO = Robot.isReal() ? new ActuatorShoulderIOReal() : new ActuatorShoulderIOSim();
          actuatorWristIO = Robot.isReal() ? new ActuatorWristIOReal() : new ActuatorWristIOSim();
          intake = new IntakeSubsystem(actuatorShoulderIO, actuatorWristIO);
          rollerIO = Robot.isReal() ? new RollerSubsystemIOTalon() : new RollerSubsystemIOSim();
          roller = new RollerSubsystem(rollerIO);
          elevatorIO = Robot.isReal() ? new ElevatorIOTalonFX() : new ElevatorIOSim();
          elevator = new ElevatorSubsystem(elevatorIO);     
    }

    // Instantiate missing subsystems
    if (swerve == null) {
      swerve = new SwerveSubsystem(
              SwerveSubsystem.createVisionIOs(),
              new GyroIO() {},
              SwerveSubsystem.createModuleIOs());
    }

    if (intake == null) {
      intake = new IntakeSubsystem(new ActuatorShoulderIO(){}, new ActuatorWristIO(){});
    }
    if (roller == null) {
      roller = new RollerSubsystem(new RollerSubsystemIO() {});
    }
    if (elevator == null) {
      elevator = new ElevatorSubsystem(new ElevatorIO() {});
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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
    autoChooser.addOption(
        "Demo Auton", new PathPlannerAuto("Demo Auton"));
    autoChooser.addOption(
        "Top Auto 2nd Top Note", new PathPlannerAuto("Top Auto 2nd Top Note"));
    autoChooser.addOption(
        "Top Auto Top Note", new PathPlannerAuto("Top Auto Top Note"));
    autoChooser.addOption(
        "1 + 2 + 1 Top Auto", new PathPlannerAuto("1 + 2 + 1 Top Auto"));
    autoChooser.addOption(
        "KILL ME", swerve.runVelocityCmd(()-> new ChassisSpeeds(1.0,0.0,0.0)).withTimeout(0.1));

    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    // DO NOT DELETE 
    // DO NOT DELETE     // DO NOT DELETE     // DO NOT DELETE     // DO NOT DELETE     // DO NOT DELETE     // DO NOT DELETE     // DO NOT DELETE 
        // DO NOT DELETE     // DO NOT DELETE     // DO NOT DELETE     // DO NOT DELETE     // DO NOT DELETE     // DO NOT DELETE 
            // DO NOT DELETE     // DO NOT DELETE     // DO NOT DELETE     // DO NOT DELETE 
    // if (aprilTagVision == null) {
    //   // In replay, match the number of instances for each robot
    //   switch (Constants.getRobot()) {
    //     case ROBOT_2023C:
    //       aprilTagVision =
    //           new AprilTagVision(
    //               new AprilTagVisionIO() {},
    //               new AprilTagVisionIO() {},
    //               new AprilTagVisionIO() {},
    //               new AprilTagVisionIO() {});
    //       break;
    //     case ROBOT_2023P:
    //       aprilTagVision = new AprilTagVision(new AprilTagVisionIO() {});
    //       break;
    //     default:
    //       aprilTagVision = new AprilTagVision();
    //       break;
      // }
    // }
    
    configureBindings();
  }

  private void configureBindings() {
    if(Robot.isSimulation()){
      swerve.setDefaultCommand(
          swerve.runVelocityFieldRelative(
              () ->
                  new ChassisSpeeds(
                      -modifyAxis(controller.getLeftY()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                      -modifyAxis(controller.getLeftX()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                      -modifyAxis(controller.getLeftTriggerAxis()) * SwerveSubsystem.MAX_ANGULAR_SPEED)));
    }
    else{
      swerve.setDefaultCommand(
            swerve.runVelocityFieldRelative(
                () ->
                    new ChassisSpeeds(
                        -modifyAxis(controller.getLeftY()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                        -modifyAxis(controller.getLeftX()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                        -modifyAxis(controller.getRightX()) * SwerveSubsystem.MAX_ANGULAR_SPEED)));
    }
    controller
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        swerve.setPose(
                            new Pose2d(
                                swerve.getPose().getTranslation(),
                                AllianceFlipUtil.apply(Rotation2d.fromDegrees(0)))))
                .ignoringDisable(true));
    m_driverController.a().toggleOnTrue((new RunCommand(()->elevator.setSetpoint(ElevatorConstants.SOFT_LIMIT_HEIGHT)).alongWith(new IntakeDefaultCommand(intake, ArmStates.AMP))));
    m_driverController.a().toggleOnFalse((new RunCommand(()->elevator.setSetpoint(0.1))).alongWith((new IntakeDefaultCommand(intake, ArmStates.SOURCE))));

    // intake.setDefaultCommand(new IntakeDefaultCommand(intake,ActuatorState.NEUTRAL));
    // m_driverController.a().whileTrue((new IntakeDefaultCommand(intake, ActuatorState.AMP))).onFalse(
    //   new IntakeDefaultCommand(intake, ActuatorState.NEUTRAL)
    // );
    // m_driverController.b().whileTrue(new IntakeDefaultCommand(intake, ActuatorState.TRAP)).onFalse(
    //   new IntakeDefaultCommand(intake, ActuatorState.NEUTRAL)
    // );
    // m_driverController.x().whileTrue(new IntakeDefaultCommand(intake, ActuatorState.SOURCE)).onFalse(
    //   new IntakeDefaultCommand(intake, ActuatorState.NEUTRAL)
    // );
    // m_driverController.rightBumper().toggleOnTrue(new ManualRollersCommand(roller, RollerState.INTAKE));
    // m_driverController.rightBumper().toggleOnFalse(new ManualRollersCommand(roller, RollerState.OUTTAKE));
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  private static double modifyAxis(double value, double deadband){
    value = deadband(value, deadband);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public Command getAutonomousCommand() {
    // return swerve.runVelocityCmd(()->new ChassisSpeeds(1,0,0)).withTimeout(0.5);
    return autoChooser.get();
  }

  public SendableChooser<RobotType> buildRobotChooser(){
    SendableChooser<RobotType> chooser = new SendableChooser<>();

    List<RobotType> options = RobotType.getList();

    chooser.setDefaultOption("ROBOT_2024C", RobotType.ROBOT_2024C);

    options.forEach(type -> chooser.addOption(type.name(), type));

    return chooser;
  }
}
