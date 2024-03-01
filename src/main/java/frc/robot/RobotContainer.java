// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
package frc.robot;


import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

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
import frc.robot.Constants.Intake.DesiredStates;
import frc.robot.Constants.Intake.Shoulder;
import frc.robot.Constants.Intake.DesiredStates.Amp;
import frc.robot.Constants.Intake.DesiredStates.Ground;
import frc.robot.Constants.Intake.DesiredStates.Neutral;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.commands.Autos.AutoPathHelper;
import frc.robot.commands.Elevator.ElevatorPositionCommand;
import frc.robot.commands.Intake.IntakePositionCommand;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.commands.Shooter.ShooterNeutral;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.Roller.RollerIO;
import frc.robot.subsystems.Intake.Roller.RollerIOReal;
import frc.robot.subsystems.Intake.Roller.RollerIOSim;
import frc.robot.subsystems.Intake.Shoulder.ShoulderIO;
import frc.robot.subsystems.Intake.Shoulder.ShoulderIOReal;
import frc.robot.subsystems.Intake.Shoulder.ShoulderIOSim;
import frc.robot.subsystems.Intake.Wrist.WristIO;
import frc.robot.subsystems.Intake.Wrist.WristIOReal;
import frc.robot.subsystems.Intake.Wrist.WristIOSim;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterIOReal;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.GyroIO;
import frc.robot.subsystems.Swerve.GyroIOPigeon2;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Turret.TurretIO;
import frc.robot.subsystems.Turret.TurretIOReal;
import frc.robot.subsystems.Turret.TurretIOSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.NoteVisualizer;

public class RobotContainer {

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

  private final LoggedDashboardChooser<String> autoChooser; // = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
  private final LoggedDashboardChooser<RobotType> robotChooser = new LoggedDashboardChooser<>("Robot Choices", buildRobotChooser());

  public static ShoulderIO shoulderIO = null;// = Robot.isReal() ? new ActuatorShoulderIOReal() : new ActuatorShoulderIOSim();
  public static WristIO wristIO = null; // = Robot.isReal() ? new ActuatorWristIOReal() : new ActuatorWristIOSim();
  public static RollerIO rollerIO = null; // = Robot.isReal() ? new RollerSubsystemIOTalon() : new RollerSubsystemIOSim();
  public static IntakeSubsystem intake = null; // = new IntakeSubsystem(actuatorShoulderIO, actuatorWristIO);
  public static ElevatorIO elevatorIO = null;//  = Robot.isReal() ? new ElevatorIOTalonFX() : new ElevatorIOSim();
  public static ElevatorSubsystem elevator = null;// = new ElevatorSubsystem(elevatorIO);
  ShooterIO shooterIO = Robot.isReal() ? new ShooterIOReal() : new ShooterIOSim();
  TurretIO turretIO = Robot.isReal() ? new TurretIOReal() : new TurretIOSim();
  ShooterSubsystem shooter = new ShooterSubsystem(shooterIO, turretIO);

  public final static CommandXboxController m_driverController = new CommandXboxController(0);

  public static SuperStructureVisualizer viz = new SuperStructureVisualizer(
    "SuperStructure", null, ()-> elevator.getSecondStageLength() ,()->elevator.getAverageExtension(), 
    ()->Math.toDegrees(intake.shoulderGetRads() - (Math.PI/2.0)), () -> Math.toDegrees(intake.wristGetRads()-(Math.PI/2.0))); //TODO: FIX to make visualizer work

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
          shoulderIO = Robot.isReal() ? new ShoulderIOReal() : new ShoulderIOSim();
          wristIO = Robot.isReal() ? new WristIOReal() : new WristIOSim();
          rollerIO = Robot.isReal() ? new RollerIOReal() : new RollerIOSim();
          intake = new IntakeSubsystem(shoulderIO, wristIO, rollerIO);
          elevatorIO = Robot.isReal() ? new ElevatorIOTalonFX() : new ElevatorIOSim();
          elevator = new ElevatorSubsystem(elevatorIO);
          shooterIO = Robot.isReal() ? new ShooterIOReal() : new ShooterIOSim();
          turretIO = Robot.isReal() ? new TurretIOReal() : new TurretIOSim();
          shooter = new ShooterSubsystem(shooterIO, turretIO); 
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
          shoulderIO = Robot.isReal() ? new ShoulderIOReal() : new ShoulderIOSim();
          wristIO = Robot.isReal() ? new WristIOReal() : new WristIOSim();
          rollerIO = Robot.isReal() ? new RollerIOReal() : new RollerIOSim();
          intake = new IntakeSubsystem(shoulderIO, wristIO, rollerIO);
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
      intake = new IntakeSubsystem(new ShoulderIOSim(){}, new WristIOSim(){}, new RollerIOSim(){});  //FIXME: This shouldn't need to be SIM, but intake is null for whatever reason
    }
    if (elevator == null) {
      elevator = new ElevatorSubsystem(new ElevatorIO() {});
    }
    if(shooter == null){
      shooter = new ShooterSubsystem(new ShooterIO() {}, new TurretIO() {});
    }

    NoteVisualizer.setRobotPoseSupplier(()->swerve.getPose());
    NoteVisualizer.setArmAngleSupplier(()-> new Rotation2d(shooter.pivotRad()));
    NoteVisualizer.setTurretAngleSupplier(()-> new Rotation2d(shooter.turretRad()));
    NamedCommands.registerCommand("Shoot", new ShooterCommand(shooter, () -> swerve.getPose(), () -> swerve.getFieldRelativeSpeeds()));
    NamedCommands.registerCommand("Intake: Ground Deploy", new IntakePositionCommand(intake, Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE).alongWith(Commands.runOnce(() -> elevator.setSetpoint(0.0))));
    NamedCommands.registerCommand("Intake: Neutral", new IntakePositionCommand(intake, Neutral.SHOULDER_ANGLE, Neutral.WRIST_ANGLE).alongWith(Commands.runOnce(() -> elevator.setSetpoint(0.0))));
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", buildAutoChooser());
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "PID Translation Auton", new PathPlannerAuto("PID Translation Auton"));
    // autoChooser.addOption(
    //     "PID Rotation Auton", new PathPlannerAuto("PID Rotation Auton"));
    // autoChooser.addOption(
    //     "Demo Auton", new PathPlannerAuto("Demo Auton"));
    // autoChooser.addOption(
    //   "Demo Auton Choreo", new PathPlannerAuto("DemoAutonChoreo"));

    // autoChooser.addOption(
    //     "Top Auto 2nd Top Note", new PathPlannerAuto("Top Auto 2nd Top Note"));
    // autoChooser.addOption(
    //     "Top Auto Top Note", new PathPlannerAuto("Top Auto Top Note"));
    // autoChooser.addOption(
    //     "1 + 2 + 1 Top Auto", new PathPlannerAuto("1 + 2 + 1 Top Auto"));
    // autoChooser.addOption(
    //     "KILL ME", swerve.runVelocityCmd(()-> new ChassisSpeeds(1.0,0.0,0.0)).withTimeout(0.1));

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
                      -modifyAxis(m_driverController.getLeftY()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                      -modifyAxis(m_driverController.getLeftX()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                      -modifyAxis(m_driverController.getLeftTriggerAxis()) * SwerveSubsystem.MAX_ANGULAR_SPEED)));
    }
    else{
      swerve.setDefaultCommand(
            swerve.runVelocityFieldRelative(
                () ->
                    new ChassisSpeeds(
                        -modifyAxis(m_driverController.getLeftY()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                        -modifyAxis(m_driverController.getLeftX()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                        -modifyAxis(m_driverController.getRightX()) * SwerveSubsystem.MAX_ANGULAR_SPEED)));
    }

    shooter.setDefaultCommand(new ShooterNeutral(shooter, ()-> false));
    // elevator.setDefaultCommand(Commands.run(()->elevator.setSetpoint(0.0), elevator));
    m_driverController
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        swerve.setPose(
                            new Pose2d(
                                swerve.getPose().getTranslation(),
                                (Rotation2d.fromDegrees(0)))))
                .ignoringDisable(true));
    // m_driverController.a().toggleOnTrue((new RunCommand(()->elevator.setSetpoint(ElevatorConstants.SOFT_LIMIT_HEIGHT)).alongWith(new IntakeDefaultCommand(intake, ArmStates.AMP))));
    // m_driverController.a().toggleOnFalse((new RunCommand(()->elevator.setSetpoint(0.1))).alongWith((new IntakeDefaultCommand(intake, ArmStates.SOURCE))));

    // // intake.setDefaultCommand(new IntakeDefaultCommand(intake,ActuatorState.NEUTRAL));
    m_driverController.b().whileTrue((new IntakePositionCommand(intake, Amp.SHOULDER_ANGLE, Amp.WRIST_ANGLE))).onFalse(
      new IntakePositionCommand(intake, Neutral.SHOULDER_ANGLE, Neutral.WRIST_ANGLE)
    );
    // m_driverController.b().whileTrue(new IntakeDefaultCommand(intake, Intake.DesiredStates.ArmStates.TRAP)).onFalse(
    //   new IntakeDefaultCommand(intake, Intake.DesiredStates.ArmStates.NEUTRAL)
    // );
    // m_driverController.x().whileTrue(new IntakeDefaultCommand(intake, Intake.DesiredStates.ArmStates.SOURCE)).onFalse(
    //   new IntakeDefaultCommand(intake, Intake.DesiredStates.ArmStates.NEUTRAL)
    // );

    // // m_driverController.a().onTrue((new ShooterCommand(shooter, ()-> swerve.getPose(), ()->swerve.getFieldRelativeSpeeds())));


    // m_driverController.rightBumper().toggleOnTrue(new ManualRollersCommand(roller, RollerState.INTAKE));
    // m_driverController.rightBumper().toggleOnFalse(new ManualRollersCommand(roller, RollerState.OUTTAKE));

        // controller.x().onTrue(shooter.run(()->shooter.setPivotPDF(Math.toRadians(30),0)));

    // m_driverController.b().onTrue(shooter.shooterVoltage(3, 1));
    shooter.setDefaultCommand(shooter.shooterVoltage(0, 0));
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
    // ArrayList<ChoreoTrajectory> fullPath = Choreo.getTrajectoryGroup("DemoAutonPath");
    // Command fullPathCommand = Commands.runOnce(()-> swerve.setPose(AllianceFlipUtil.apply(fullPath.get(0).getInitialPose())));
    // for (ChoreoTrajectory traj : fullPath) {
    //   Command trajCommand = AutoPathHelper.choreoCommand(traj, swerve);
    //   fullPathCommand = fullPathCommand.andThen(AutoPathHelper.doPathAndIntakeThenShoot(trajCommand, swerve, shooter, intake, Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE));
    // }
    // fullPathCommand = fullPathCommand.andThen(new ShooterCommand(shooter, ()->swerve.getPose(), ()->swerve.getFieldRelativeSpeeds()));
    
    //return AutoPathHelper.sequencePaths(swerve,map.toArray(new Command[]{}));
    //return AutoPathHelper.choreoCommand(Choreo.getTrajectory("DemoAutonPath"), swerve).beforeStarting(Commands.runOnce(()-> swerve.setPose(AllianceFlipUtil.apply(fullPath.get(0).getInitialPose()))));
    return buildAuton(autoChooser.get(), !autoChooser.get().contains("Bottom Path"));
  }

  public SendableChooser<RobotType> buildRobotChooser(){
    SendableChooser<RobotType> chooser = new SendableChooser<>();

    List<RobotType> options = RobotType.getList();

    chooser.setDefaultOption("ROBOT_2024C", RobotType.ROBOT_2024C);

    options.forEach(type -> chooser.addOption(type.name(), type));

    return chooser;
  }
  public Command buildAuton(String trajName, boolean preLoad) {
    ArrayList<ChoreoTrajectory> fullPath = Choreo.getTrajectoryGroup(trajName);
    ChoreoTrajectory firstTrajectory = fullPath.size() > 0 ? fullPath.get(0) : new ChoreoTrajectory();
    Command fullPathCommand = Commands.runOnce(() -> swerve.setPose(new Pose2d()));
    if (fullPath.size() > 0) {
      fullPathCommand = Commands.runOnce(()-> swerve.setPose(AllianceFlipUtil.apply(firstTrajectory.getInitialPose())));
        if (!preLoad) {
          fullPathCommand = Commands.runOnce(()-> swerve.setPose(AllianceFlipUtil.apply(firstTrajectory.getInitialPose())));
          fullPathCommand = fullPathCommand.andThen(AutoPathHelper.choreoCommand(firstTrajectory, swerve));
          if (fullPath.size() > 0) fullPath.remove(0);
        }
        for (ChoreoTrajectory traj : fullPath) {
          Command trajCommand = AutoPathHelper.choreoCommand(traj, swerve);
          fullPathCommand = fullPathCommand.andThen(AutoPathHelper.doPathAndIntakeThenShoot(trajCommand, swerve, shooter, intake, Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE));
        }
    }
    fullPathCommand = fullPathCommand.andThen(new ShooterCommand(shooter, ()->swerve.getPose(), ()->swerve.getFieldRelativeSpeeds()));
    return fullPathCommand;
  }
  public SendableChooser<String> buildAutoChooser() {
    SendableChooser<String> out = new SendableChooser<String>();
    out.addOption("DemoAutonPath", "DemoAutonPath");
    out.addOption("4NoteStart", "4NoteStart");
    out.addOption("123 Top End", "123 Top End");
    out.addOption("Bottom Path (No Preload)", "Bottom Path No Preload");
    out.addOption("Bottom Path (Score Preload)", "Bottom Path Score Preload");
    out.addOption("132 Top End", "132 Top End");
    return out;
  }
}
