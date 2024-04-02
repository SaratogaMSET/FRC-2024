// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
package frc.robot;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.Intake.DesiredStates.Amp;
import frc.robot.Constants.Intake.DesiredStates.Ground;
import frc.robot.Constants.Intake.DesiredStates.Neutral;
import frc.robot.Constants.Elevator;
import frc.robot.Constants.Mode;
import frc.robot.Constants.RobotType;
import frc.robot.Constants.ShooterFlywheelConstants;
import frc.robot.commands.Autos.AutoPathHelper;
import frc.robot.commands.Intake.IntakeNeutralCommand;
import frc.robot.commands.Intake.IntakePositionCommand;
import frc.robot.commands.Intake.MotionMagicIntakePosition;
import frc.robot.commands.Intake.RollerCommand;
import frc.robot.commands.Intake.RollerDefaultCommand;
import frc.robot.commands.LEDs.AquamarineCommand;
import frc.robot.commands.Shooter.AimTestCommand;
import frc.robot.commands.Shooter.ShooterNeutral;
import frc.robot.subsystems.Elevator.ElevatorIO;
import frc.robot.subsystems.Elevator.ElevatorIOSim;
import frc.robot.subsystems.Elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.Roller.RollerIO;
import frc.robot.subsystems.Intake.Roller.RollerIOReal;
import frc.robot.subsystems.Intake.Roller.RollerIOSim;
import frc.robot.subsystems.Intake.Roller.RollerSubsystem;
import frc.robot.subsystems.Intake.Shoulder.ShoulderIO;
import frc.robot.subsystems.Intake.Shoulder.ShoulderIOReal;
import frc.robot.subsystems.Intake.Shoulder.ShoulderIOSim;
import frc.robot.subsystems.Intake.Wrist.WristIO;
import frc.robot.subsystems.Intake.Wrist.WristIOReal;
import frc.robot.subsystems.Intake.Wrist.WristIOSim;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterIOReal;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.Shooter.ShooterParameters;
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
  // private final LoggedDashboardChooser<Double> delayChooser;
  private final LoggedDashboardChooser<Command> autoChooser; // = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
  private final LoggedDashboardChooser<RobotType> robotChooser = new LoggedDashboardChooser<>("Robot Choices", buildRobotChooser());
  public static ShoulderIO shoulderIO = null;// = Robot.isReal() ? new ActuatorShoulderIOReal() : new ActuatorShoulderIOSim();
  public static WristIO wristIO = null; // = Robot.isReal() ? new ActuatorWristIOReal() : new ActuatorWristIOSim();
  public static RollerIO rollerIO = null; // = Robot.isReal() ? new RollerSubsystemIOTalon() : new RollerSubsystemIOSim();
  public static IntakeSubsystem intake = null; // = new IntakeSubsystem(actuatorShoulderIO, actuatorWristIO);
  public static ElevatorIO elevatorIO = null;//  = Robot.isReal() ? new ElevatorIOTalonFX() : new ElevatorIOSim();
  public static ElevatorSubsystem elevator = null;// = new ElevatorSubsystem(elevatorIO);
  public static RollerSubsystem roller = null;
  public static LEDSubsystem led = new LEDSubsystem();
  public static boolean previousIntakeTriggered = false;
  public static boolean previousShooterTriggered = false;
  public static boolean gunnerRightBumper = false;
  ShooterIO shooterIO = Robot.isReal() ? new ShooterIOReal() : new ShooterIOSim();
  TurretIO turretIO = Robot.isReal() ? new TurretIOReal() : new TurretIOSim();
  ShooterSubsystem shooter = new ShooterSubsystem(shooterIO, turretIO);

  public final static CommandXboxController m_driverController = new CommandXboxController(0);
  public final static CommandXboxController gunner = new CommandXboxController(1);

  public static SuperStructureVisualizer viz = new SuperStructureVisualizer(
    "SuperStructure", null, ()-> elevator.getSecondStageLength() ,()->elevator.getAverageExtension(), 
    ()->Math.toDegrees(intake.shoulderGetRads() - (Math.PI/2.0)), () -> Math.toDegrees(intake.wristGetRads()-(Math.PI/2.0))); //TODO: FIX to make visualizer work
  // Trigger prerev = new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(),()->gunner.rightBumper().getAsBoolean());
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
          shoulderIO = Robot.isReal() ? new ShoulderIOReal() : new ShoulderIOSim();
          wristIO = Robot.isReal() ? new WristIOReal() : new WristIOSim();
          rollerIO = Robot.isReal() ? new RollerIOReal() : new RollerIOSim();
          roller = new RollerSubsystem(rollerIO);
          intake = new IntakeSubsystem(shoulderIO, wristIO);
          elevatorIO = Robot.isReal() ? new ElevatorIOTalonFX() : new ElevatorIOSim();
          elevator = new ElevatorSubsystem(elevatorIO);     
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
          roller = new RollerSubsystem(rollerIO);
          intake = new IntakeSubsystem(shoulderIO, wristIO);
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
          roller = new RollerSubsystem(rollerIO);
          intake = new IntakeSubsystem(shoulderIO, wristIO);
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
      intake = new IntakeSubsystem(new ShoulderIOSim(){}, new WristIOSim(){});  //FIXME: This shouldn't need to be SIM, but intake is null for whatever reason
    }
    if (elevator == null) {
      elevator = new ElevatorSubsystem(new ElevatorIO() {});
    }
    if(shooter == null){
      shooter = new ShooterSubsystem(new ShooterIO() {}, new TurretIO() {});
    }
    if(roller == null){
      roller = new RollerSubsystem(new RollerIO() {});
    }

    NoteVisualizer.setRobotPoseSupplier(()->swerve.getPose());
    NoteVisualizer.setArmAngleSupplier(()-> new Rotation2d(shooter.pivotRad()));
    NoteVisualizer.setTurretAngleSupplier(()-> new Rotation2d(shooter.turretRad()));
    // NamedCommands.registerCommand("Shoot", new ShooterCommand(shooter, () -> swerve.getPose(), () -> swerve.getFieldRelativeSpeeds()));
    // NamedCommands.registerCommand("Intake: Ground Deploy", new IntakePositionCommand(intake, Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE).alongWith(Commands.runOnce(() -> elevator.setSetpoint(0.0))));
    // NamedCommands.registerCommand("Intake: Neutral", new IntakePositionCommand(intake, Neutral.SHOULDER_ANGLE, Neutral.WRIST_ANGLE).alongWith(Commands.runOnce(() -> elevator.setSetpoint(0.0))));
    // autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // delayChooser = new LoggedDashboardChooser<>("Delay Choices", delayChooser());
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", buildAutoChooser());


    Commands.runOnce(()->elevator.resetEncoders(), elevator);
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
          swerve.runVelocityTeleopFieldRelative(
              () ->
                  new ChassisSpeeds(
                      -modifyAxis(m_driverController.getLeftY()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                      -modifyAxis(m_driverController.getLeftX()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                      -modifyAxis(m_driverController.getLeftTriggerAxis()) * SwerveSubsystem.MAX_ANGULAR_SPEED)
                      ));
    }
    else{
      swerve.setDefaultCommand(
            swerve.runVelocityTeleopFieldRelative(
                () ->
                    new ChassisSpeeds(
                        -modifyAxis(m_driverController.getLeftY()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                        -modifyAxis(m_driverController.getLeftX()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                        -modifyAxis(m_driverController.getRightX()) * SwerveSubsystem.MAX_ANGULAR_SPEED)
                        ));
    }

    // intake.setDefaultCommand(Commands.run(()->intake.setWristVoltage(0.5)));
    intake.setDefaultCommand(new IntakeNeutralCommand(intake, () -> gunner.povUp().getAsBoolean()));
    shooter.setDefaultCommand(new ShooterNeutral(shooter, roller, ()->gunner.rightBumper().getAsBoolean(),()-> m_driverController.rightTrigger().getAsBoolean()));
    roller.setDefaultCommand(new RollerDefaultCommand(roller, () -> intake.shoulderGetRads()));
    led.setDefaultCommand(led.deleteEverything());
    // elevator.setDefaultCommand(Commands.run(()->elevator.setSetpoint(0.0), elevator));
    // m_driverController
    //     .y()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->{
    //                     swerve.setPose(
    //                         new Pose2d(
    //                             swerve.getPose().getTranslation(),
    //                             (Rotation2d.fromDegrees(0))));
    //                     // swerve.setYaw(new Rotation2d(0.0));
    //                 })
    //             .ignoringDisable(true));
       m_driverController
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->{
                        swerve.zeroGyro();
                    })
                .ignoringDisable(true));


    // m_driverController.a().toggleOnFalse((new RunCommand(()->elevator.setSetpoint(0.1))).alongWith((new IntakeDefaultCommand(intake, ArmStates.SOURCE))));
    // m_driverController.b().whileTrue((new IntakePositionCommand(intake, Amp.SHOULDER_ANGLE, Amp.WRIST_ANGLE)));s
    Command groundIntakeToShooter = new IntakePositionCommand(intake, Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE)
    .alongWith(
      new RollerCommand(roller, 3, true, ()->intake.shoulderGetRads()));
    /* onFalse complement of groundIntakeToShooter */
    Command groundIntakeToNeutral = new RollerCommand(roller, -1, false, ()->intake.shoulderGetRads()).withTimeout(0.14);
    gunner.rightBumper().whileTrue(Commands.run(() -> {gunnerRightBumper = true; Logger.recordOutput("GunnerRightBumper", gunnerRightBumper);})).whileFalse(Commands.run(() -> {gunnerRightBumper = false; Logger.recordOutput("GunnerRightBumper", gunnerRightBumper);}));

    m_driverController.rightBumper().whileTrue(groundIntakeToShooter).onFalse(groundIntakeToNeutral);
    m_driverController.rightTrigger().whileTrue(
      // Commands.repeatingSequence(
      new IntakePositionCommand(intake, Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE)
    .alongWith(
      new RollerCommand(roller, 9, false, ()->intake.shoulderGetRads()))
        .alongWith(
          Commands.run(
            ()->
            new ConditionalCommand(
            shooter.setShooterState(0, 0, 44),
            shooter.setShooterState(ShooterParameters.mps_to_voltage(9), 0, 44), 
          ()-> (gunner.getLeftY()>-0.5)
            ),
            shooter
        )
        
        
      .alongWith((Commands.run(()->elevator.setSetpoint(0), elevator)))));
 
    m_driverController.leftBumper().whileTrue(new RollerCommand(roller, 5, false, ()->intake.shoulderGetRads())).onFalse(new RollerCommand(roller, 0.0, false, ()->intake.shoulderGetRads())
      .until(()->roller.getShooterBeamBreak()));
    
    m_driverController.a().whileTrue(Commands.run(()-> roller.setShooterFeederVoltage(12), roller)).onFalse(Commands.runOnce(()->roller.setShooterFeederVoltage(0.0), roller));


    gunner.y().whileTrue(new AimTestCommand(shooter, ()-> new Pose2d(AllianceFlipUtil.apply(ShooterFlywheelConstants.subwoofer.getTranslation()), swerve.getRotation()), ()-> new ChassisSpeeds(0.0,0.0,0.0), roller, false, 13, true, true, false));

    gunner.x().whileTrue(new AimTestCommand(shooter, ()-> new Pose2d(AllianceFlipUtil.apply(ShooterFlywheelConstants.podium.getTranslation()), swerve.getRotation()), ()-> new ChassisSpeeds(0.0,0.0,0.0), roller, true, 13, true, true, false));

    gunner.a().whileTrue(new AimTestCommand(shooter, ()-> new Pose2d(AllianceFlipUtil.apply(ShooterFlywheelConstants.bluelineinner328.getTranslation()), swerve.getRotation()), ()-> new ChassisSpeeds(0.0,0.0,0.0), roller, true, 14.5, true, true, false));

    // gunner.a().whileTrue(new AimTestCommand(shooter, ()-> swerve.getPose(), ()-> swerve.getFieldRelativeSpeeds(), roller, true, 9.5, true, true, true));

    gunner.leftBumper().whileTrue(new AimTestCommand(shooter, ()-> new Pose2d(AllianceFlipUtil.apply(ShooterFlywheelConstants.podium.getTranslation()), swerve.getRotation()), ()-> new ChassisSpeeds(0.0,0.0,0.0), roller, true, 10.5, false, true, false));

    // gunner.b().whileTrue(Commands.run(() -> shooter.setTurretProfiled(Units.degreesToRadians(-45), 0), shooter));

    // gunner.leftBumper().toggleOnTrue((Commands.run(()->elevator.setSetpoint(Elevator.HangHeight), elevator)).alongWith(new ShooterNeutral(shooter)));
    // gunner.rightBumper().toggleOnTrue(Commands.run(()->elevator.setSetpoint(Elevator.ClimbHeight), elevator).alongWith(new ShooterNeutral(shooter)));

    // gunner.leftBumper().whileTrue(Commands.run(()->elevator.setVoltage(3, 3), elevator)).onFalse(
    //   Commands.runOnce(()->elevator.setVoltage(0, 0), elevator)
    // );

    // gunner.leftBumper().whileTrue(Commands.run(()-> elevator.setSetpoint(Elevator.ClimbHeight))
    // .alongWith(
      // elevator.flipOut()
    // )
    // );

    // gunner.rightBumper().whileTrue(Commands.run(()->elevator.setVoltage(-3, -3), elevator)).onFalse(
    //   Commands.runOnce(()->elevator.setVoltage(0, 0), elevator)
    // );

    gunner.leftTrigger().whileTrue(new RollerCommand(roller, -3, false, ()->intake.shoulderGetRads()));
    gunner.rightTrigger().toggleOnTrue(new IntakePositionCommand(intake, Amp.SHOULDER_ANGLE, Amp.WRIST_ANGLE).alongWith(Commands.run(()->elevator.setSetpoint(Amp.elevatorPosition), elevator)))
    .toggleOnFalse(Commands.run(()->elevator.setSetpoint(0), elevator));
    // m_driverController.rightBumper().toggleOnTrue(new IntakePositionCommand(intake, Amp.SHOULDER_ANGLE, Amp.WRIST_ANGLE).alongWith(Commands.run(()->elevator.setSetpoint(Amp.elevatorPosition), elevator)));


    new Trigger(
      ()-> (roller.getIntakeBeamBreak() && !previousIntakeTriggered)
      )
      .onTrue(
          Commands.run(
            ()-> {
              m_driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
              gunner.getHID().setRumble(RumbleType.kBothRumble, 1.0);
              previousIntakeTriggered = roller.getIntakeBeamBreak();
              led.color(0, 255, 255);
            }).withTimeout(0.3).andThen(
              ()->{
              previousIntakeTriggered = roller.getIntakeBeamBreak();
              m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
              gunner.getHID().setRumble(RumbleType.kBothRumble, 0.0);
              led.color(0, 0, 0);
            })
            ).onFalse(
              Commands.run(() -> {
                previousIntakeTriggered = roller.getIntakeBeamBreak();
                m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                gunner.getHID().setRumble(RumbleType.kBothRumble, 0.0);
              }
              )
            );

      new Trigger(
      ()-> (roller.getShooterBeamBreak() && !previousShooterTriggered)
      // ()-> m_driverController.rightTrigger().getAsBoolean()
      )
      .onTrue(
          Commands.run(
            ()-> {
              m_driverController.getHID().setRumble(RumbleType.kRightRumble, 1.0);
              gunner.getHID().setRumble(RumbleType.kRightRumble, 1.0);
              previousShooterTriggered = roller.getShooterBeamBreak();
              led.color(0, 255, 0);
            }).withTimeout(0.3).andThen(
              ()->{
              previousShooterTriggered = roller.getShooterBeamBreak();
              m_driverController.getHID().setRumble(RumbleType.kRightRumble, 0.0);
              gunner.getHID().setRumble(RumbleType.kRightRumble, 0.0);
              // SmartDashboard.putNumber("find me Rumble has ended2", Timer.getFPGATimestamp());
              led.color(0,0,0);
            })
            ).onFalse(
              Commands.run(() -> {
                previousShooterTriggered = roller.getShooterBeamBreak();
                m_driverController.getHID().setRumble(RumbleType.kRightRumble, 0.0);
                gunner.getHID().setRumble(RumbleType.kRightRumble, 0.0);}
              )
            );

       new Trigger(
        ()-> (shooter.shooterInputs.shooterAppliedVolts[0] > 4.5)
      // ()-> m_driverController.rightTrigger().getAsBoolean()
      )
      .onTrue(
          Commands.run(
            ()-> {
              gunner.getHID().setRumble(RumbleType.kLeftRumble, 1.0);
            })
            ).onFalse(
              Commands.run(() -> {
                gunner.getHID().setRumble(RumbleType.kLeftRumble, 0.0);}
              )
            );

    // new Trigger(
    //   ()-> m_driverController.rightTrigger().getAsBoolean()
    // ).onTrue(
    //   new ConditionalCommand(shooter.setShooterState(0, 0, 44), shooter.setShooterState(ShooterParameters.mps_to_voltage(9), 0, 44), ()->gunner.rightBumper().getAsBoolean())
    // );

  
    //THE BELOW BUTTONS ARE SIM OR FOR TUNING ONLY: DO NOT RUN ON AT A COMPETITION
    //REMEMBER TO COMMENT THEM OUT AND BRING THE REAL RESPECTIVE BUTTONS BACK
    

    // gunner.a().whileTrue(AutoPathHelper.pathfindToPose(new Pose2d(AllianceFlipUtil.apply(FieldConstants.NotePositions.ampScoringPosition), Rotation2d.fromDegrees(90)), swerve));
    // gunner.y().toggleOnTrue(Commands.run(()->elevator.setSetpoint(Elevator.ClimbHeight), elevator).alongWith(new ShooterNeutral(shooter)));
    // gunner.x().toggleOnTrue(Commands.run(()->elevator.setSetpoint(Elevator.HangHeight), elevator).alongWith(new ShooterNeutral(shooter)));
    // gunner.a().toggleOnTrue((new IntakePositionCommand(intake, Amp.SHOULDER_ANGLE, Amp.WRIST_ANGLE).alongWith(Commands.run(()->elevator.setSetpoint(Amp.elevatorPosition), elevator))));

      



    // m_driverController.b().whileTrue(shooter.turretVoltage(1.0)).whileFalse(shooter.turretVoltage(0));
    // m_driverController.a().whileTrue(shooter.turretVoltage(-1.0)).whileFalse(shooter.turretVoltage(0));
    // m_driverController.x().whileTrue(shooter.turretAngleDegrees(0)).whileFalse(shooter.turretVoltage(0));
    // m_driverController.b().whileTrue(shooter.pivotVoltage(1.0)).whileFalse(shooter.pivotVoltage(0));
    // m_driverController.a().whileTrue(shooter.pivotVoltage(-1.0)).whileFalse(shooter.pivotVoltage(0));
    // m_driverController.x().whileTrue(shooter.pivotAngleDegrees(Constants.ShooterPivotConstants.kHigherBound)).whileFalse(shooter.pivotVoltage(0));
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
    // return buildAuton(autoChooser.get(), !(autoChooser.get().contains("Bottom Path") || autoChooser.get().contains("Basic")) , delayChooser.get());
    swerve.setDriveCurrentLimit(120);
    return autoChooser.get();

    // switch(autoChooser.get()){
    //   case "Just Leave":
    //   case "Just Shoot":
    //   case "1 Piece + Mobility Middle Subwoofer":
    //   //SP: -0.0029714447844025804 0.8779510235995609 0.18793980509970054
        
    //   case "1 Piece + Mobility Amp-side Subwoofer":
    //   case "1 Piece + Mobility Enemy Source side Subwoofer":

    //   case "2 Piece Middle Subwoofer":
        
      // case "Drive SysId (Quasistatic Forward)":
      //   return swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward).withTimeout(15);
      // case "Drive SysId (Quasistatic Reverse)":
      //   return swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).withTimeout(15);
      // case "Drive SysId (Dynamic Forward)":
      //   return swerve.sysIdDynamic(SysIdRoutine.Direction.kForward).withTimeout(15);
      // case "Drive SysId (Dynamic Reverse)":
      //   return swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse).withTimeout(15);
      // default:
        // return Commands.sequence(
        //   Commands.runOnce(()->swerve.setPose(AllianceFlipUtil.apply(ShooterFlywheelConstants.subwoofer)), swerve),
        //   new WaitCommand(delayChooser.get()),
        //   Commands.parallel(
        //     new AimTestCommand(swerve, shooter, ()-> AllianceFlipUtil.apply(ShooterFlywheelConstants.subwoofer),()-> swerve.getFieldRelativeSpeeds(), roller, false, 9.0, true, false),
        //     Commands.sequence(
        //       new WaitCommand(2),
        //       Commands.run(()-> roller.setShooterFeederVoltage(12), roller)
        //     )
        //   ).withTimeout(3),
        //     (new RollerCommand(roller, 0.0, false, ()->intake.shoulderGetRads())).withTimeout(0.01),
        //     Commands.run(()->swerve.runVelocity(new ChassisSpeeds(2.0,0.0,0.0)),swerve).withTimeout(1)
        //   );
        // if(autoChooser.get().equals("2 Piece Middle Subwoofer") 
        //   || autoChooser.get().equals("Just Leave") 
        //   || autoChooser.get().equals("Just Shoot") 
        //   || autoChooser.get().equals("1 Piece + Mobility Middle Subwoofer") 
        //   || autoChooser.get().equals("1 Piece + Mobility Enemy Source side Subwoofer")
        //   || autoChooser.get().equals("1 Piece + Mobility Amp-side Subwoofer")){
        //     return new WaitCommand(0);
        // }
        // else{
          // return new WaitCommand(0.0);
        // }
    // }
  }

  public SendableChooser<RobotType> buildRobotChooser(){
    SendableChooser<RobotType> chooser = new SendableChooser<>();

    List<RobotType> options = RobotType.getList();

    chooser.setDefaultOption("ROBOT_2024C", RobotType.ROBOT_2024C);

    options.forEach(type -> chooser.addOption(type.name(), type));

    return chooser;
  }
  public SendableChooser<Double> delayChooser(){
    SendableChooser<Double> chooser = new SendableChooser<>();
    chooser.setDefaultOption("0.0", 0.0);
    for (double i = 0.5; i < 15; i+=0.5) {
      chooser.addOption(i + "", i);
    }
    return chooser;
  }
  public Command buildAuton(String trajName, boolean preLoad, double delay) {
    ArrayList<ChoreoTrajectory> fullPath = Choreo.getTrajectoryGroup(trajName);
    ChoreoTrajectory firstTrajectory = fullPath.size() > 0 ? fullPath.get(0) : new ChoreoTrajectory(); 
    Command fullPathCommand = Commands.runOnce(()-> swerve.setPose(AllianceFlipUtil.apply(firstTrajectory.getInitialPose())))
    .andThen(Commands.runOnce(()-> swerve.setTurnState(
      swerve.kinematics.toSwerveModuleStates(fullPath.get(0).getInitialState().getChassisSpeeds())), swerve));
    if (delay != 0.0) fullPathCommand = fullPathCommand.andThen(new WaitCommand(delay));
    if (fullPath.size() > 0) {
        if (!preLoad) {
          fullPathCommand = fullPathCommand.andThen(AutoPathHelper.followPathWhileIntaking(AutoPathHelper.choreoCommand(firstTrajectory, swerve, trajName), intake, Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE));
          fullPath.remove(0);
        }
        for (ChoreoTrajectory traj : fullPath) {
          Command trajCommand = AutoPathHelper.choreoCommand(traj, swerve, trajName);
          // fullPathCommand = fullPathCommand.andThen(AutoPathHelper.doPathAndIntakeThenShoot(trajCommand, swerve, shooter, intake, Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE, roller));
          fullPathCommand = fullPathCommand.andThen(AutoPathHelper.doPathAndIntakeThenShoot(trajCommand, swerve, shooter, intake, roller, traj.getTotalTime()));
        }
    }
    fullPathCommand = fullPathCommand.andThen(Commands.parallel(
                new AimTestCommand(shooter, ()-> swerve.getPose(), ()-> swerve.getFieldRelativeSpeeds(), roller, true, 9.0, true, false, false)
                // new SequentialCommandGroup(
                //     new WaitCommand(1),
                //     Commands.run(()-> roller.setShooterFeederVoltage(12), roller)
                // ))
                .withTimeout(2))
                .andThen(
                  Commands.parallel( 
                    shooter.setShooterState(0, 0, 0),
                    (new RollerCommand(roller, 0.0, false, ()->intake.shoulderGetRads())).withTimeout(0.01)
                  )
                )
          );
    return fullPathCommand;
  }
  public Command buildAutonGrief(String trajName, boolean preLoad, double delay) {
    ArrayList<ChoreoTrajectory> fullPath = Choreo.getTrajectoryGroup(trajName);
    ChoreoTrajectory firstTrajectory = fullPath.size() > 0 ? fullPath.get(0) : new ChoreoTrajectory(); 
    Command fullPathCommand = Commands.runOnce(()-> swerve.setPose(AllianceFlipUtil.apply(firstTrajectory.getInitialPose())))
    .andThen(Commands.runOnce(()-> swerve.setTurnState(
      swerve.kinematics.toSwerveModuleStates(fullPath.get(0).getInitialState().getChassisSpeeds())), swerve));
    if (delay != 0.0) fullPathCommand = fullPathCommand.andThen(new WaitCommand(delay));
    if (fullPath.size() > 0) {
        if (!preLoad) {
          fullPathCommand = fullPathCommand.andThen(AutoPathHelper.followPathWhileIntaking(AutoPathHelper.choreoCommand(firstTrajectory, swerve, trajName), intake, Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE));
          fullPath.remove(0);
        }
        for (ChoreoTrajectory traj : fullPath) {
          Command trajCommand = AutoPathHelper.choreoCommand(traj, swerve, trajName);
          // fullPathCommand = fullPathCommand.andThen(AutoPathHelper.doPathAndIntakeThenShoot(trajCommand, swerve, shooter, intake, Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE, roller));
          fullPathCommand = fullPathCommand.andThen(AutoPathHelper.doPathAndIntakeThenExtake(trajCommand, swerve, shooter, intake, roller, traj.getTotalTime()));
        }
    }
    fullPathCommand = fullPathCommand.beforeStarting(Commands.parallel(
                new AimTestCommand(shooter, ()-> swerve.getPose(), ()-> swerve.getFieldRelativeSpeeds(), roller, true, 9.0, true, false, false)
                // new SequentialCommandGroup(
                //     new WaitCommand(1),
                //     Commands.run(()-> roller.setShooterFeederVoltage(12), roller)
                // ))
                .withTimeout(2))
                .andThen(
                  Commands.parallel( 
                    shooter.setShooterState(0, 0, 0),
                    (new RollerCommand(roller, 0.0, false, ()->intake.shoulderGetRads())).withTimeout(0.01)
                  )
                )
          );
    return fullPathCommand;
  }
  public SendableChooser<Command> buildAutoChooser() {
    SendableChooser<Command> out = new SendableChooser<Command>();
    out.setDefaultOption("1 Piece + Mobility Middle Subwoofer", oneMiddleSubwoofer(0.0));
    out.addOption("Just Leave", justLeave(0.0));
    out.addOption("Just Shoot", justShoot(0.0));
    out.addOption("1 Piece + Mobility Amp-side Subwoofer", oneAmpSide(0.0));
    out.addOption("1 Piece + Mobility Enemy Source side Subwoofer", oneEnemySource(0.0));
    out.addOption("2 Piece Middle Subwoofer", twoPieceCommand(0.0));
    // out.addOption("Source Side Griefer", sourceSideGrief());s
    // out.addOption("Source Side 1 Note", );
    
    out.addOption(
        "Drive SysId (Quasistatic Forward)",
        swerve.sysIdQuasistatic(Direction.kForward));
    out.addOption(
        "Drive SysId (Quasistatic Reverse)",
        swerve.sysIdQuasistatic(Direction.kReverse));
    out.addOption(
        "Drive SysId (Dynamic Forward)",swerve.sysIdDynamic(Direction.kForward));
    out.addOption(
        "Drive SysId (Dynamic Reverse)", swerve.sysIdDynamic(Direction.kReverse));
    // out.addOption("2 Note Speaker Side", "2 Note Speaker Side");
    // // out.addOption("3 Note Speaker Side", "3 Note Speaker Side");
    out.addOption("4 Note Speaker Side", buildAuton("4 Note Speaker Side", true, 0));
    // out.addOption("3 Note Source Side Score Preload", "3 Note Source Side Score Preload"); 

    // out.addOption("2 Note Speaker Side", "2 Note Speaker Side");
    // out.addOption("DemoAutonPath", "DemoAutonPath");
    // out.addOption("4NoteStart", "4NoteStart");

    // out.addOption("Basic Mobility", "Basic Mobility");
    // out.addOption("PID Translation", "PID Translation");
    // out.setDefaultOption("Top Path 123", "Top Path 123");
    // out.addOption("Top Path 132", "Top Path 132");
    // out.addOption("Top Path Mid First 123", "Top Path Mid First 123");

    // out.addOption("Middle Path 43", "Middle Path 43");
    // out.addOption("Middle Path 34", "Middle Path 34");

    // out.addOption("Bottom Path (No Preload)", "Bottom Path No Preload");
    // out.addOption("Bottom Path (Score Preload)", "Bottom Path Score Preload");


    return out;
  }
  
  public Command justLeave(double wait){
    return Commands.sequence(
          Commands.runOnce(()->swerve.setPose(AllianceFlipUtil.apply(ShooterFlywheelConstants.subwoofer)), swerve),
          new WaitCommand(wait),
          Commands.run(()->swerve.runVelocity(new ChassisSpeeds(2.0,0.0,0.0)),swerve).withTimeout(1.25),
          Commands.run(()->swerve.runVelocity(new ChassisSpeeds(0.0,0.0,0.0)),swerve).withTimeout(1)
        ).alongWith(shooter.setShooterState(0, 0, 0));
  }

  public Command oneMiddleSubwoofer(double wait){
    return Commands.sequence(
          Commands.runOnce(()->swerve.setPose(AllianceFlipUtil.apply(ShooterFlywheelConstants.subwoofer)), swerve),
          new WaitCommand(wait),
          Commands.parallel(
            //new ShooterCommand(shooter, ()-> AllianceFlipUtil.apply(ShooterFlywheelConstants.subwoofer),()-> new ChassisSpeeds(0.0,0.0,0.0), roller, false, 9.0),
            new AimTestCommand(shooter, ()-> AllianceFlipUtil.apply(ShooterFlywheelConstants.subwoofer),()-> new ChassisSpeeds(0.0,0.0,0.0), roller, true, 9.0, true, false, false),
            Commands.sequence(
              new WaitCommand(2),
              Commands.run(()-> roller.setShooterFeederVoltage(12), roller)
            )
          ).withTimeout(3),
          Commands.race( 
            shooter.setShooterState(0, 0, 0),
            (new RollerCommand(roller, 0.0, false, ()->intake.shoulderGetRads())),
            Commands.run(()->swerve.runVelocity(new ChassisSpeeds(2.0,0.0,0.0)),swerve).withTimeout(1)
          ),
          Commands.run(()->swerve.runVelocity(new ChassisSpeeds(0.0,0.0,0.0)), swerve)
          );
  }
  public Command justShoot(double wait){
    return Commands.sequence(
          Commands.runOnce(()->swerve.setPose(AllianceFlipUtil.apply(ShooterFlywheelConstants.subwoofer)), swerve),
          new WaitCommand(wait),
          Commands.parallel(
            //new ShooterCommand(shooter, ()-> AllianceFlipUtil.apply(ShooterFlywheelConstants.subwoofer),()-> new ChassisSpeeds(0.0,0.0,0.0), roller, false, 9.0),
            new AimTestCommand(shooter, ()-> AllianceFlipUtil.apply(ShooterFlywheelConstants.subwoofer),()-> new ChassisSpeeds(0.0,0.0,0.0), roller, true, 9.0, true, false, false),
            Commands.sequence(
              new WaitCommand(2),
              Commands.run(()-> roller.setShooterFeederVoltage(12), roller)
            )
          ).withTimeout(3),
          Commands.parallel( 
            shooter.setShooterState(0, 0, 0),
            (new RollerCommand(roller, 0.0, false, ()->intake.shoulderGetRads())).withTimeout(0.01)));
  }
  public Command oneAmpSide(double wait){
     ArrayList<ChoreoTrajectory> ampFullPath = Choreo.getTrajectoryGroup("Amp Back Out");
        Command fullPathCommand = Commands.runOnce(()-> swerve.setPose(AllianceFlipUtil.apply(ampFullPath.get(0).getInitialPose())));
        // Command ampPath = AutoPathHelper.choreoCommand(ampFullPath.get(0), swerve, "Amp Back Out");
        for(ChoreoTrajectory traj: ampFullPath){
          fullPathCommand = fullPathCommand.andThen(AutoPathHelper.choreoCommand(traj, swerve, "Amp Back Out"));        
        }
        return Commands.sequence(
            Commands.runOnce(()->swerve.setPose(AllianceFlipUtil.apply(ShooterFlywheelConstants.ampside)), swerve),
            new WaitCommand(wait),
            Commands.parallel(//new ShooterCommand(shooter, ()-> AllianceFlipUtil.apply(ShooterFlywheelConstants.ampside),()-> swerve.getFieldRelativeSpeeds(), roller, false, 9.0),
              new AimTestCommand(shooter, ()-> AllianceFlipUtil.apply(ShooterFlywheelConstants.subwoofer),()-> new ChassisSpeeds(0.0,0.0,0.0), roller, true, 9.0, true, false, false),
              Commands.sequence(
                new WaitCommand(2),
                Commands.run(()-> roller.setShooterFeederVoltage(12), roller)
              )
            ).withTimeout(3),
              Commands.parallel( 
                shooter.setShooterState(0, 0, 0),
                (new RollerCommand(roller, 0.0, false, ()->intake.shoulderGetRads())).withTimeout(0.01),
                fullPathCommand
          )
            );
  }
  public Command twoPieceCommand(double wait){
     return Commands.sequence(

          Commands.runOnce(()->swerve.setPose(AllianceFlipUtil.apply(ShooterFlywheelConstants.subwoofer)), swerve),

          new WaitCommand(wait),

         Commands.parallel(
              new AimTestCommand(shooter, ()-> swerve.getPose(),()-> swerve.getFieldRelativeSpeeds(), roller, false, 9.0, true, false, false),

              Commands.sequence(
                new WaitCommand(2),
                Commands.run(()-> roller.setShooterFeederVoltage(12), roller)
            )
          ).withTimeout(3),

          new RollerCommand(roller, 0.0, false, ()->intake.shoulderGetRads()).withTimeout(0.01),
          
          Commands.race(
            // shooter.anglingDegrees(0.0,44),

            Commands.run(()->swerve.runVelocity(new ChassisSpeeds(0.5,0.0,0.0)),swerve).withTimeout(3),

            new IntakePositionCommand(intake, Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE)
              .alongWith(
            new RollerCommand(roller, 6, false, ()->intake.shoulderGetRads()).alongWith(shooter.anglingDegrees(0.0,44))
            )
          ),

          Commands.runOnce(()->swerve.runVelocity(new ChassisSpeeds(0,0.0,0.0)),swerve),

          Commands.parallel(
              new AimTestCommand(shooter, ()-> swerve.getPose(),()-> swerve.getFieldRelativeSpeeds(), roller, false, 9.0, true, false, false),

              Commands.sequence(
                new WaitCommand(2),
                Commands.run(()-> roller.setShooterFeederVoltage(12), roller)
            )
          ).withTimeout(3),

          Commands.parallel( 
            shooter.setShooterState(0, 0, 0),
            (new RollerCommand(roller, 0.0, false, ()->intake.shoulderGetRads())).withTimeout(0.01)
          )

        );
  }
  public Command oneEnemySource(double wait){
      ArrayList<ChoreoTrajectory> fullPath = Choreo.getTrajectoryGroup("Source Back out");
        Command path = AutoPathHelper.choreoCommand(fullPath.get(0), swerve, "Source Back out");
        return Commands.sequence(
            Commands.runOnce(()->swerve.setPose(AllianceFlipUtil.apply(fullPath.get(0).getInitialPose())), swerve),
            new WaitCommand(wait),
            Commands.parallel(//new ShooterCommand(shooter, ()-> AllianceFlipUtil.apply(ShooterFlywheelConstants.ampside),()-> swerve.getFieldRelativeSpeeds(), roller, false, 9.0),
              new AimTestCommand(shooter, ()-> AllianceFlipUtil.apply(ShooterFlywheelConstants.subwoofer),()-> new ChassisSpeeds(0.0,0.0,0.0), roller, true, 9.0, true, false, false),
              Commands.sequence(
                new WaitCommand(2),
                Commands.run(()-> roller.setShooterFeederVoltage(12), roller)
              )
            ).withTimeout(3),
             Commands.parallel( 
              shooter.setShooterState(0, 0, 0),
              (new RollerCommand(roller, 0.0, false, ()->intake.shoulderGetRads())).withTimeout(0.01),
              path
          )
            );
  }
  public Command sourceSideGrief(){
    // ArrayList<ChoreoTrajectory> fullPath = Choreo.getTrajectoryGroup("Source Side Grief");
    return buildAutonGrief("Source Side Grief", true, 0);
    
  }
}