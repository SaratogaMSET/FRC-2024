// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//
package frc.robot;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.DesiredStates.Ground;
import frc.robot.Constants.Intake.DesiredStates.Neutral;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ShooterFlywheelConstants;
import frc.robot.commands.Autos.AutoPathHelper;
import frc.robot.commands.Intake.IntakeNeutralCommand;
import frc.robot.commands.Intake.IntakePositionCommand;
import frc.robot.commands.Intake.RollerCommand;
import frc.robot.commands.Intake.RollerDefaultCommand;
import frc.robot.commands.Intake.TrapCommand;
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
import frc.robot.subsystems.LEDs.LEDSubsystem.State;
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
import frc.robot.util.EqualsUtil;
import frc.robot.util.NoteVisualizer;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private SwerveSubsystem swerve = null;
  private final LoggedDashboardChooser<Double> delayChooser;
  private final LoggedDashboardChooser<Command> autoChooser;
  public static ShoulderIO shoulderIO = null;
  public static WristIO wristIO = null;
  public static RollerIO rollerIO = null;
  public static IntakeSubsystem intake = null;
  public static ElevatorIO elevatorIO = null;
  public static ElevatorSubsystem elevator = null;
  public static RollerSubsystem roller = null;
  public static LEDSubsystem led = new LEDSubsystem();
  public static boolean previousIntakeTriggered = false;
  public static boolean previousShooterTriggered = false;
  public static boolean gunnerRightBumper = false;
  ShooterIO shooterIO = Robot.isReal() ? new ShooterIOReal() : new ShooterIOSim();
  TurretIO turretIO = Robot.isReal() ? new TurretIOReal() : new TurretIOSim();
  ShooterSubsystem shooter = new ShooterSubsystem(shooterIO, turretIO);

  public static final CommandXboxController m_driverController = new CommandXboxController(0);
  public static final CommandXboxController gunner = new CommandXboxController(1);

  public static SuperStructureVisualizer viz =
      !Robot.isReal() // Only for Simulation
          ? new SuperStructureVisualizer(
              "SuperStructure",
              null,
              () -> elevator.getSecondStageLength(),
              () -> elevator.getAverageExtension(),
              () -> Math.toDegrees(intake.shoulderGetRads() - (Math.PI / 2.0)),
              () -> Math.toDegrees(intake.wristGetRads() - (Math.PI / 2.0)))
          : null; // TODO: FIX to make visualizer work

  public RobotContainer() {

    // Instantiate active subsystems
    if (Constants.getMode() != Mode.REPLAY && Robot.isReal()) {
      switch (Constants.getRobot()) {
        case ROBOT_2024C:
          swerve =
              new SwerveSubsystem(
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
          swerve =
              new SwerveSubsystem(
                  Robot.isReal()
                      ? SwerveSubsystem.createCamerasReal()
                      : SwerveSubsystem.createCamerasSim(),
                  Robot.isReal() ? new GyroIOPigeon2() : new GyroIO() {},
                  Robot.isReal()
                      ? SwerveSubsystem.createTalonFXModules()
                      : SwerveSubsystem.createSimModules());
          break;

          // THIS DOES NOTHING
        case ROBOT_SIMBOT:
          swerve =
              new SwerveSubsystem(
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
          swerve =
              new SwerveSubsystem(
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
      swerve =
          new SwerveSubsystem(
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
      swerve =
          new SwerveSubsystem(
              SwerveSubsystem.createVisionIOs(),
              new GyroIO() {},
              SwerveSubsystem.createModuleIOs());
    }

    if (intake == null) intake = new IntakeSubsystem(new ShoulderIOSim() {}, new WristIOSim() {});
    if (elevator == null) elevator = new ElevatorSubsystem(new ElevatorIO() {});
    if (shooter == null) shooter = new ShooterSubsystem(new ShooterIO() {}, new TurretIO() {});
    if (roller == null) roller = new RollerSubsystem(new RollerIO() {});

    AutoPathHelper.createFactory(swerve);

    NoteVisualizer.setRobotPoseSupplier(() -> swerve.getPose());
    NoteVisualizer.setArmAngleSupplier(() -> new Rotation2d(shooter.pivotRad()));
    NoteVisualizer.setTurretAngleSupplier(() -> new Rotation2d(shooter.turretRad()));

    delayChooser = new LoggedDashboardChooser<>("Delay Choices", delayChooser());
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", buildAutoChooser());

    Commands.runOnce(
        () -> elevator.resetEncoders(),
        elevator); // Zero encoders based on start(is this a good idea?)

    configureBindings();
  }

  private void configureBindings() {

    // Default Commands
    if (Robot.isSimulation()) {
      swerve.setDefaultCommand(
          swerve.runVelocityTeleopFieldRelative(
              () ->
                  new ChassisSpeeds(
                      -modifyAxis(m_driverController.getLeftY()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                      -modifyAxis(m_driverController.getLeftX()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                      -modifyAxis(m_driverController.getLeftTriggerAxis())
                          * SwerveSubsystem.MAX_ANGULAR_SPEED)));
    } else {
      swerve.setDefaultCommand(
          swerve.runVelocityTeleopFieldRelative(
              () ->
                  new ChassisSpeeds(
                      -modifyAxis(m_driverController.getLeftY()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                      -modifyAxis(m_driverController.getLeftX()) * SwerveSubsystem.MAX_LINEAR_SPEED,
                      -modifyAxis(m_driverController.getRightX())
                          * SwerveSubsystem.MAX_ANGULAR_SPEED)));
    }

    intake.setDefaultCommand(
        new IntakeNeutralCommand(intake, () -> (gunner.getHID().getPOV() == 90)));

    shooter.setDefaultCommand(
        new ShooterNeutral(
            shooter,
            roller,
            () -> gunner.getHID().getRightBumper(),
            () ->
                m_driverController
                    .getHID()
                    .rightTrigger(0.5, CommandScheduler.getInstance().getDefaultButtonLoop())
                    .getAsBoolean()));

    roller.setDefaultCommand(new RollerDefaultCommand(roller, () -> intake.shoulderGetRads()));

    m_driverController
        .y()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      swerve.setPose(
                          new Pose2d(
                              swerve.getPose().getTranslation(),
                              AllianceFlipUtil.apply(new Rotation2d(0.0))));
                    })
                .ignoringDisable(true)
                .withName("Zero Gyro"));

    /* AMP */
    m_driverController
        .rightBumper()
        .whileTrue(groundIntakeAndRoller(3, true).withName("Intake for Amp"))
        .onFalse(
            runRollers(-1, false)
                .withTimeout(0.14)
                .withName("Magic Correction for Amp")); // Magic Timer

    /* Rev Logic */
    m_driverController
        .rightTrigger()
        .whileTrue(
            groundIntakeAndRoller(9, false)
                .alongWith(
                    new ConditionalCommand(
                        shooter.setShooterStateMPS(9, 0, 44),
                        shooter.setShooterState(0, 0, 44),
                        () -> (gunnerRightBumper)))
                .alongWith((Commands.run(() -> elevator.setSetpoint(0), elevator)))
                .withName("Ground Intake"));

    /* What on earth is this command */
    m_driverController
        .leftBumper()
        .whileTrue(runRollers(5, false).withName("Rollers 5v"))
        .onFalse(
            runRollers(0, false)
                .until(() -> roller.getShooterBeamBreak())
                .withName("Spin down after rolling"));

    /* Shoot */
    m_driverController
        .a()
        .whileTrue(Commands.run(() -> roller.setShooterFeederVoltage(12), roller).withName("Shoot"))
        .onFalse(
            Commands.runOnce(() -> roller.setShooterFeederVoltage(0.0), roller)
                .withName("Spin down after shot"));

    // In place
    gunner
        .y()
        .whileTrue(
            aimPresetGyroStationary(ShooterFlywheelConstants.subwoofer, 13)
                .alongWith(new IntakePositionCommand(intake, Neutral.shoulderAvoidTurretAngle, 0))
                .withName("Subwoofer Aim"));

    gunner
        .x()
        .whileTrue(
            aimPresetGyroStationary(ShooterFlywheelConstants.podium, 13)
                .alongWith(new IntakePositionCommand(intake, Neutral.shoulderAvoidTurretAngle, 0))
                .withName("Podium Aim"));

    gunner
        .a()
        .whileTrue(
            aimPresetGyroStationary(ShooterFlywheelConstants.bluelineinner328, 14.5)
                .alongWith(new IntakePositionCommand(intake, Neutral.shoulderAvoidTurretAngle, 0))
                .withName("Feeder Aim"));

    gunner
        .b()
        .whileTrue(
            aimRobotGyro(14)
                .alongWith(new IntakePositionCommand(intake, Neutral.shoulderAvoidTurretAngle, 0))
                .withName("Vision Aim"));

    gunner
        .leftBumper()
        .whileTrue(
            aimPresetGyroStationary(ShooterFlywheelConstants.podium, 9.5)
                .alongWith(new IntakePositionCommand(intake, Neutral.shoulderAvoidTurretAngle, 0))
                .withName("Gunner podium Aim"));

    // gunner.pov(270)
    //   .whileTrue(new AimTestCommand(shooter,
    //       () -> new
    // Pose2d(AllianceFlipUtil.apply(ShooterFlywheelConstants.podium.getTranslation()),
    //           swerve.getRotation()),
    //       swerve::emptyChassisSpeeds), roller, true, 10.5, false, true, false,
    // (ShooterParameters.mps_to_kRPM(-0.5) * 1000))
    //       .alongWith(new IntakePositionCommand(intake, Neutral.shoulderAvoidTurretAngle, 0)));

    // gunner.b().whileTrue(Commands.run(() ->
    // shooter.setTurretProfiled(Units.degreesToRadians(-45), 0), shooter));

    // gunner.leftBumper().toggleOnTrue((Commands.run(()->elevator.setSetpoint(Elevator.HangHeight),
    // elevator)).alongWith(new ShooterNeutral(shooter)));
    // gunner.rightBumper().toggleOnTrue(Commands.run(()->elevator.setSetpoint(Elevator.ClimbHeight),
    // elevator).alongWith(new ShooterNeutral(shooter)));

    // gunner.leftBumper().whileTrue(Commands.run(()->elevator.setVoltage(3, 3),
    // elevator)).onFalse(
    // Commands.runOnce(()->elevator.setVoltage(0, 0), elevator)
    // );

    // gunner.leftBumper().whileTrue(Commands.run(()->
    // elevator.setSetpoint(Elevator.ClimbHeight))
    // .alongWith(
    // elevator.flipOut()
    // )
    // );

    // gunner.rightBumper().whileTrue(Commands.run(()->elevator.setVoltage(-3, -3),
    // elevator)).onFalse(
    // Commands.runOnce(()->elevator.setVoltage(0, 0), elevator)
    // );

    gunner.leftTrigger().whileTrue(runRollers(-1, false).withName("Extake"));
    // new RollerCommand(roller, -1, false, () -> intake.shoulderGetRads()));
    // gunner.rightTrigger()
    //     .toggleOnTrue(new IntakePositionCommand(intake, Amp.SHOULDER_ANGLE, Amp.WRIST_ANGLE)
    //         .alongWith(Commands.run(() -> elevator.setSetpoint(Amp.elevatorPosition), elevator)))
    //     .toggleOnFalse(Commands.run(() -> elevator.setSetpoint(0), elevator));

    gunner.back().whileTrue(runRollers(1, false).withName("Rollers 1v"));
    // new RollerCommand(roller, 1, false, () -> intake.shoulderGetRads()));

    gunner
        .povUp()
        .whileTrue(Commands.run(() -> elevator.setVoltage(9, 9)).withName("Elevator Up"))
        .onFalse(
            Commands.run(() -> elevator.setVoltage(0.2, 0.2))
                .withName("Hold Elevator In Place(ks + kg)"));
    gunner
        .povDown()
        .whileTrue(Commands.run(() -> elevator.setVoltage(-9, -9)).withName("Elevator Down"))
        .onFalse(Commands.run(() -> elevator.setVoltage(0, 0)).withName("No power to elevator"));

    gunner
        .start()
        .whileTrue(new TrapCommand(intake, roller, elevator, 2.3, 2.0).withName("Trap Command"));
    // m_driverController.rightBumper().toggleOnTrue(new
    // IntakePositionCommand(intake, Amp.SHOULDER_ANGLE,
    // Amp.WRIST_ANGLE).alongWith(Commands.run(()->elevator.setSetpoint(Amp.elevatorPosition),
    // elevator)));

    /* Start LED and Rumble Triggers */

    // new Trigger(() -> (roller.getIntakeBeamBreak() && !previousIntakeTriggered))
    //     .onTrue(
    //         Commands.run(
    //                 () -> {
    //                   maxRumble();
    //                   previousIntakeTriggered = roller.getIntakeBeamBreak();
    //                   // led.color(0, 255, 255);
    //                 })
    //             .withTimeout(0.3)
    //             .andThen(
    //                 () -> {
    //                   previousIntakeTriggered = roller.getIntakeBeamBreak();
    //                   stopRumble();
    //                   // led.color(0, 0, 0);
    //                 }))
    //     .onFalse(
    //         Commands.run(
    //             () -> {
    //               previousIntakeTriggered = roller.getIntakeBeamBreak();
    //               stopRumble();
    //             }));

    /* Shooter Beam Break */
    new Trigger(() -> (roller.getShooterBeamBreak() && !previousShooterTriggered))
        .and(() -> (!shooter.shooterReady()))
        .onTrue(
            rumbleCommand()
                .withTimeout(0.3)
                .beforeStarting(() -> previousShooterTriggered = roller.getShooterBeamBreak())
                .finallyDo(() -> previousShooterTriggered = roller.getShooterBeamBreak()));

    /* Ready to Shoot */
    new Trigger(() -> (shooter.shooterReady()))
        .whileTrue(rumbleCommand())
        .onTrue( // This indeed does work
            Commands.either(
                    led.setStateCommand(State.SHOOTING_W_VISION),
                    led.setStateCommand(State.SHOOTING_WOUT_VISION),
                    swerve::getIsVisionTargetSeen)
                .withName("Shooter Revving States"))
        .onFalse(led.setStateCommand(State.NORMAL));
    /* Has Note */
    new Trigger(() -> roller.getCarriageBeamBreak() || roller.getShooterBeamBreak())
        .and(() -> (!shooter.shooterReady()))
        .onTrue(
            Commands.run(() -> System.out.println("Has Note"))
                .andThen(led.setStateCommand(State.HAS_NOTE))
                .withName("HAS NOTE"))
        .onFalse(led.setStateCommand(State.NORMAL));

    if (Robot.isSimulation()) {
      /* if we're intaking */
      new Trigger(() -> roller.getIntakeVoltage() > 0)
          .and(
              () ->
                  EqualsUtil.epsilonEquals(
                      intake.shoulderGetRads(),
                      Constants.Intake.DesiredStates.Ground.LOWER_MOTION_SHOULDER_ANGLE,
                      0.1))
          .whileTrue(Commands.run(() -> NoteVisualizer.takeAutoNote()));
    }

    // THE BELOW BUTTONS ARE SIM OR FOR TUNING ONLY: DO NOT RUN ON AT A COMPETITION
    // REMEMBER TO COMMENT THEM OUT AND BRING THE REAL RESPECTIVE BUTTONS BACK

    // gunner.a().whileTrue(AutoPathHelper.pathfindToPose(new
    // Pose2d(AllianceFlipUtil.apply(FieldConstants.NotePositions.ampScoringPosition),
    // Rotation2d.fromDegrees(90)), swerve));
    // gunner.y().toggleOnTrue(Commands.run(()->elevator.setSetpoint(Elevator.ClimbHeight),
    // elevator).alongWith(new ShooterNeutral(shooter)));
    // gunner.x().toggleOnTrue(Commands.run(()->elevator.setSetpoint(Elevator.HangHeight),
    // elevator).alongWith(new ShooterNeutral(shooter)));
    // gunner.a().toggleOnTrue((new IntakePositionCommand(intake,
    // Amp.SHOULDER_ANGLE,
    // Amp.WRIST_ANGLE).alongWith(Commands.run(()->elevator.setSetpoint(Amp.elevatorPosition),
    // elevator))));

    // m_driverController.b().whileTrue(shooter.turretVoltage(1.0)).whileFalse(shooter.turretVoltage(0));
    // m_driverController.a().whileTrue(shooter.turretVoltage(-1.0)).whileFalse(shooter.turretVoltage(0));
    // m_driverController.x().whileTrue(shooter.turretAngleDegrees(0)).whileFalse(shooter.turretVoltage(0));
    // m_driverController.b().whileTrue(shooter.pivotVoltage(1.0)).whileFalse(shooter.pivotVoltage(0));
    // m_driverController.a().whileTrue(shooter.pivotVoltage(-1.0)).whileFalse(shooter.pivotVoltage(0));
    // m_driverController.x().whileTrue(shooter.pivotAngleDegrees(Constants.ShooterPivotConstants.kHigherBound)).whileFalse(shooter.pivotVoltage(0));
  }

  public Command rumbleCommand() {
    return Commands.startEnd(
        () -> {
          m_driverController.getHID().setRumble(RumbleType.kRightRumble, 1.0);
          gunner.getHID().setRumble(RumbleType.kRightRumble, 1.0);
        },
        () -> {
          m_driverController.getHID().setRumble(RumbleType.kRightRumble, 0.0);
          gunner.getHID().setRumble(RumbleType.kRightRumble, 0.0);
        });
  }

  public Command groundIntakeAndRoller(double rollerVoltage, boolean amp) {
    return new IntakePositionCommand(
            intake, Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE)
        .alongWith(runRollers(rollerVoltage, amp));
  }

  public Command runRollers(double voltage, boolean ampIntake) {
    return new RollerCommand(roller, voltage, ampIntake, intake::shoulderGetRads);
  }

  public Command intakePositionCommand(double shoulderRadians, double wristRadians) {
    return Commands.runOnce(
            () -> {
              intake.setPreviousZeroed(false);
            })
        .andThen(
            Commands.run(
                () -> {
                  intake.setAngleShoulderMotionMagic(shoulderRadians);
                  if (Math.abs(
                          Math.toDegrees(intake.shoulderGetRads())
                              - Math.toDegrees(shoulderRadians))
                      <= Intake.Shoulder.POSITION_ERROR_TOLERANCE) {
                    intake.setAngleWrist(wristRadians);
                  }
                },
                intake));
  }

  // TODO: Double check if its robot relative
  public Command aimRobotGyro(double vMag) {
    return shooter.aimTestCommandFactory(
        swerve::getPose, swerve::getFieldRelativeSpeeds, true, vMag, true, true, false, 0);
  }

  public Command aimRobotGyroStationary(double vMag) {
    return shooter.aimTestCommandFactory(
        swerve::getPose, swerve::emptyChassisSpeeds, true, vMag, true, true, false, 0);
  }

  public Command aimRobotGyroAuto(double vMag) {
    return shooter.aimTestCommandFactory(
        swerve::getPose, swerve::getFieldRelativeSpeeds, true, vMag, true, false, false, 0);
  }

  public Command aimRobotGyroStationaryAuto(double vMag) {
    return shooter.aimTestCommandFactory(
        swerve::getPose, swerve::emptyChassisSpeeds, true, vMag, true, false, false, 0);
  }

  /* Aims from passed in setpoint, applying AllianceFlipUtil */
  public Command aimPresetGyroStationary(Pose2d robotPose, double vMag) {
    return shooter.aimTestCommandFactory(
        () -> new Pose2d(AllianceFlipUtil.apply(robotPose.getTranslation()), swerve.getRotation()),
        swerve::emptyChassisSpeeds,
        true,
        vMag,
        true,
        true,
        false,
        0);
  }

  /* Aims from passed in setpoint, applying AllianceFlipUtil */
  public Command aimPresetGyroStationaryAuto(Pose2d robotPose, double vMag) {
    return shooter.aimTestCommandFactory(
        () -> new Pose2d(AllianceFlipUtil.apply(robotPose.getTranslation()), swerve.getRotation()),
        swerve::emptyChassisSpeeds,
        true,
        vMag,
        true,
        false,
        false,
        0);
  }

  public static Command aimRobotGyro(
      double vMag, SwerveSubsystem swerve, ShooterSubsystem shooter) {
    return shooter.aimTestCommandFactory(
        swerve::getPose, swerve::getFieldRelativeSpeeds, true, vMag, true, true, false, 0);
  }

  public static Command aimRobotGyroStationary(
      double vMag, SwerveSubsystem swerve, ShooterSubsystem shooter) {
    return shooter.aimTestCommandFactory(
        swerve::getPose, swerve::emptyChassisSpeeds, true, vMag, true, true, false, 0);
  }

  public static Command aimRobotGyroAuto(
      double vMag, SwerveSubsystem swerve, ShooterSubsystem shooter) {
    return shooter.aimTestCommandFactory(
        swerve::getPose, swerve::getFieldRelativeSpeeds, true, vMag, true, false, false, 0);
  }

  public static Command aimRobotGyroStationaryAuto(
      double vMag, SwerveSubsystem swerve, ShooterSubsystem shooter) {
    return shooter.aimTestCommandFactory(
        swerve::getPose, swerve::emptyChassisSpeeds, true, vMag, true, false, false, 0);
  }

  /* Aims from passed in setpoint, applying AllianceFlipUtil */
  public static Command aimPresetGyroStationary(
      Pose2d robotPose, double vMag, SwerveSubsystem swerve, ShooterSubsystem shooter) {
    return shooter.aimTestCommandFactory(
        () -> new Pose2d(AllianceFlipUtil.apply(robotPose.getTranslation()), swerve.getRotation()),
        swerve::emptyChassisSpeeds,
        true,
        vMag,
        true,
        true,
        false,
        0);
  }

  /* Aims from passed in setpoint, applying AllianceFlipUtil */
  public static Command aimPresetGyroStationaryAuto(
      Pose2d robotPose, double vMag, SwerveSubsystem swerve, ShooterSubsystem shooter) {
    return shooter.aimTestCommandFactory(
        () -> new Pose2d(AllianceFlipUtil.apply(robotPose.getTranslation()), swerve.getRotation()),
        swerve::emptyChassisSpeeds,
        true,
        vMag,
        true,
        false,
        false,
        0);
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

  private static double modifyAxis(double value, double deadband) {
    value = deadband(value, deadband);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public Command getAutonomousCommand() {
    // return buildAuton(autoChooser.get(), !(autoChooser.get().contains("Bottom
    // Path") || autoChooser.get().contains("Basic")) , delayChooser.get());
    // swerve.setDriveCurrentLimit(120);
    // try{
    //   return autoChooser.get()
    //       .beforeStarting(new WaitCommand(delayChooser.get()));
    // }
    // catch(Exception e){
    return autoChooser.get();
    // }
    // switch(autoChooser.get()){
    // case "Just Leave":
    // case "Just Shoot":
    // case "1 Piece + Mobility Middle Subwoofer":
    // //SP: -0.0029714447844025804 0.8779510235995609 0.18793980509970054

    // case "1 Piece + Mobility Amp-side Subwoofer":
    // case "1 Piece + Mobility Enemy Source side Subwoofer":

    // case "2 Piece Middle Subwoofer":

    // case "Drive SysId (Quasistatic Forward)":
    // return
    // swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward).withTimeout(15);
    // case "Drive SysId (Quasistatic Reverse)":
    // return
    // swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).withTimeout(15);
    // case "Drive SysId (Dynamic Forwardl)":
    // return swerve.sysIdDynamic(SysIdRoutine.Direction.kForward).withTimeout(15);
    // case "Drive SysId (Dynamic Reverse)":
    // return swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse).withTimeout(15);
    // default:
    // return Commands.sequence(
    // Commands.runOnce(()->swerve.setPose(AllianceFlipUtil.apply(ShooterFlywheelConstants.subwoofer)),
    // swerve),
    // new WaitCommand(delayChooser.get()),
    // Commands.parallel(
    // new AimTestCommand(swerve, shooter, ()->
    // AllianceFlipUtil.apply(ShooterFlywheelConstants.subwoofer),()->
    // swerve.getFieldRelativeSpeeds(), roller, false, 9.0, true, false),
    // Commands.sequence(
    // new WaitCommand(2),
    // Commands.run(()-> roller.setShooterFeederVoltage(12), roller)
    // )
    // ).withTimeout(3),
    // (new RollerCommand(roller, 0.0, false,
    // ()->intake.shoulderGetRads())).withTimeout(0.01),
    // Commands.run(()->swerve.runVelocity(new
    // ChassisSpeeds(2.0,0.0,0.0)),swerve).withTimeout(1)
    // );
    // if(autoChooser.get().equals("2 Piece Middle Subwoofer")
    // || autoChooser.get().equals("Just Leave")
    // || autoChooser.get().equals("Just Shoot")
    // || autoChooser.get().equals("1 Piece + Mobility Middle Subwoofer")
    // || autoChooser.get().equals("1 Piece + Mobility Enemy Source side Subwoofer")
    // || autoChooser.get().equals("1 Piece + Mobility Amp-side Subwoofer")){
    // return new WaitCommand(0);
    // }
    // else{
    // return new WaitCommand(0.0);
    // }
    // }
  }

  public SendableChooser<Double> delayChooser() {
    SendableChooser<Double> chooser = new SendableChooser<>();
    chooser.setDefaultOption("0.0", 0.0);
    for (double i = 0.5; i < 15; i += 0.5) {
      chooser.addOption(i + "", i);
    }
    return chooser;
  }

  public Command driveOnlyAuton(String trajName, double delay) {
    Optional<Trajectory<SwerveSample>> fullPath = Choreo.loadTrajectory(trajName);
    var splits = fullPath.get().splits();
    Command fullPathCommand =
        Commands.runOnce(() -> swerve.setPose(AutoPathHelper.initialPose(fullPath)));
    for (int i = 0; i < splits.size(); i++) {
      Command trajCommand = AutoPathHelper.choreoCommand(swerve, trajName, i);
      fullPathCommand = fullPathCommand.andThen(trajCommand);
    }
    return fullPathCommand;
  }

  public Command buildAuton(String trajName, boolean preLoad, double delay) {
    Optional<Trajectory<SwerveSample>> fullPath = Choreo.loadTrajectory(trajName);
    var splits = fullPath.get().splits();
    Command fullPathCommand =
        Commands.runOnce(() -> swerve.setPose(AutoPathHelper.initialPose(fullPath)));

    // swerve.setYaw(firstTrajectory.getInitialPose().getRotation());

    if (delay != 0.0) fullPathCommand = fullPathCommand.andThen(new WaitCommand(delay));
    for (int i = 0; i < splits.size(); i++) {
      Command trajCommand = AutoPathHelper.choreoCommand(swerve, trajName, i);
      fullPathCommand =
          fullPathCommand.andThen(
              AutoPathHelper.shootThenPathAndIntake(trajCommand, swerve, shooter, intake, roller));
    }
    /* Final Shot */
    fullPathCommand = fullPathCommand.andThen(AutoPathHelper.shot(swerve, shooter, roller, intake));

    return fullPathCommand;
  }

  public Command buildAutonWithTriggers(String trajName) {
    return AutoPathHelper.choreoAutoWithTriggers(swerve, intake, roller, shooter, trajName);
  }

  //   public Command build4NoteAuton(String trajName, boolean preLoad, double delay) {
  //     ArrayList<ChoreoTrajectory> fullPath = Choreo.getTrajectoryGroup(trajName);
  //     ChoreoTrajectory firstTrajectory =
  //         fullPath.size() > 0 ? fullPath.get(0) : new ChoreoTrajectory();
  //     // swerve.setYaw(firstTrajectory.getInitialPose().getRotation());
  //     Command fullPathCommand =
  //         Commands.runOnce(
  //             () -> swerve.setPose(AllianceFlipUtil.apply(firstTrajectory.getInitialPose())));
  //     // .andThen(Commands.runOnce(() -> swerve.setTurnState(

  //     // swerve.setYaw(firstTrajectory.getInitialPose().getRotation());

  //     if (delay != 0.0) fullPathCommand = fullPathCommand.andThen(new WaitCommand(delay));
  //     if (fullPath.size() > 0) {
  //       // if (!preLoad) {
  //       //   fullPathCommand = fullPathCommand.andThen(
  //       //
  // AutoPathHelper.followPathWhileIntaking(AutoPathHelper.choreoCommand(firstTrajectory,
  //       // swerve, trajName),
  //       //           intake, Ground.LOWER_MOTION_SHOULDER_ANGLE,
  // Ground.LOWER_MOTION_WRIST_ANGLE));
  //       //   fullPath.remove(0);
  //       // }
  //       for (ChoreoTrajectory traj : fullPath) {
  //         Command trajCommand = AutoPathHelper.choreoCommand(traj, swerve, trajName);
  //         // fullPathCommand =
  //         // fullPathCommand.andThen(AutoPathHelper.doPathAndIntakeThenShoot(trajCommand,
  //         // swerve, shooter, intake, Ground.LOWER_MOTION_SHOULDER_ANGLE,
  //         // Ground.LOWER_MOTION_WRIST_ANGLE, roller));
  //         fullPathCommand =
  //             fullPathCommand.andThen(
  //                 AutoPathHelper.pathAndIntakeThenIntakeRevThenShotFor4Note(
  //                     trajCommand,
  //                     swerve,
  //                     shooter,
  //                     intake,
  //                     roller,
  //                     traj.getTotalTime(),
  //                     AllianceFlipUtil.apply(traj.getFinalPose())));
  //       }
  //     }
  //     fullPathCommand =
  //         fullPathCommand
  //             .beforeStarting(
  //                 Commands.parallel(
  //                         new AimTestCommand(
  //                             shooter,
  //                             () -> AllianceFlipUtil.apply(swerve.getPose()),
  //                             () -> swerve.getFieldRelativeSpeeds(),
  //                             roller,
  //                             true,
  //                             9,
  //                             true,
  //                             false,
  //                             false,
  //                             0),
  //                         new SequentialCommandGroup(
  //                             new WaitCommand(1),
  //                             Commands.run(() -> roller.setShooterFeederVoltage(11), roller)))
  //                     .withTimeout(1.2))
  //             .andThen(
  //                 Commands.parallel(
  //                     shooter.setShooterState(0, 0, 0).withTimeout(0.01),
  //                     new RollerCommand(roller, 0.0, false, () -> intake.shoulderGetRads())
  //                         .withTimeout(0.01))); // Last shot,
  //     // then
  //     // return to
  //     // neutral

  //     return fullPathCommand;
  //   }

  //   public Command build4NoteAmpAuton(String trajName, boolean preLoad, double delay) {
  //     ArrayList<ChoreoTrajectory> fullPath = Choreo.getTrajectoryGroup(trajName);
  //     ChoreoTrajectory firstTrajectory =
  //         fullPath.size() > 0 ? fullPath.get(0) : new ChoreoTrajectory();
  //     // swerve.setYaw(firstTrajectory.getInitialPose().getRotation());
  //     Command fullPathCommand =
  //         Commands.runOnce(
  //             () -> swerve.setPose(AllianceFlipUtil.apply(firstTrajectory.getInitialPose())));
  //     // .andThen(Commands.runOnce(() -> swerve.setTurnState(
  //     //
  //     //
  // swerve.kinematics.toSwerveModuleStates(fullPath.get(0).getInitialState().getChassisSpeeds())),
  // swerve) );

  //     // swerve.setYaw(firstTrajectory.getInitialPose().getRotation());

  //     if (delay != 0.0) fullPathCommand = fullPathCommand.andThen(new WaitCommand(delay));
  //     if (fullPath.size() > 0) {
  //       // if (!preLoad) {
  //       //   fullPathCommand = fullPathCommand.andThen(
  //       //
  // AutoPathHelper.followPathWhileIntaking(AutoPathHelper.choreoCommand(firstTrajectory,
  //       // swerve, trajName),
  //       //           intake, Ground.LOWER_MOTION_SHOULDER_ANGLE,
  // Ground.LOWER_MOTION_WRIST_ANGLE));
  //       //   fullPath.remove(0);
  //       // }
  //       for (ChoreoTrajectory traj : fullPath) {
  //         Command trajCommand = AutoPathHelper.choreoCommand(traj, swerve, trajName);
  //         // fullPathCommand =
  //         // fullPathCommand.andThen(AutoPathHelper.doPathAndIntakeThenShoot(trajCommand,
  //         // swerve, shooter, intake, Ground.LOWER_MOTION_SHOULDER_ANGLE,
  //         // Ground.LOWER_MOTION_WRIST_ANGLE, roller));
  //         fullPathCommand =
  //             fullPathCommand.andThen(
  //                 AutoPathHelper.pathAndIntakeThenIntakeRevThenShotFor4NoteAmp(
  //                     trajCommand, swerve, shooter, intake, roller, traj.getTotalTime()));
  //       }
  //     }
  //     fullPathCommand =
  //         fullPathCommand
  //             .beforeStarting(
  //                 Commands.parallel(
  //                         new AimTestCommand(
  //                             shooter,
  //                             () -> AllianceFlipUtil.apply(swerve.getPose()),
  //                             () -> swerve.getFieldRelativeSpeeds(),
  //                             roller,
  //                             true,
  //                             12,
  //                             true,
  //                             false,
  //                             false,
  //                             0),
  //                         new SequentialCommandGroup(
  //                             new WaitCommand(1.5),
  //                             Commands.run(() -> roller.setShooterFeederVoltage(11), roller)))
  //                     .withTimeout(1.7))
  //             .andThen(
  //                 Commands.parallel(
  //                     shooter.setShooterState(0, 0, 0).withTimeout(0.01),
  //                     new RollerCommand(roller, 0.0, false, () -> intake.shoulderGetRads())
  //                         .withTimeout(0.01))); // Last shot,
  //     // then
  //     // return to
  //     // neutral

  //     return fullPathCommand;
  //   }

  //   public Command buildNewGriefAuton(String trajName, boolean preLoad, double delay) {
  //     ArrayList<ChoreoTrajectory> fullPath = Choreo.getTrajectoryGroup(trajName);
  //     ChoreoTrajectory firstTrajectory =
  //         fullPath.size() > 0 ? fullPath.get(0) : new ChoreoTrajectory();
  //     // swerve.setYaw(firstTrajectory.getInitialPose().getRotation());
  //     Command fullPathCommand =
  //         Commands.runOnce(
  //             () -> swerve.setPose(AllianceFlipUtil.apply(firstTrajectory.getInitialPose())));
  //     // .andThen(Commands.runOnce(() -> swerve.setTurnState(
  //     //
  //     //
  // swerve.kinematics.toSwerveModuleStates(fullPath.get(0).getInitialState().getChassisSpeeds())),
  // swerve) );

  //     // swerve.setYaw(firstTrajectory.getInitialPose().getRotation());

  //     if (delay != 0.0) fullPathCommand = fullPathCommand.andThen(new WaitCommand(delay));
  //     if (fullPath.size() > 0) {
  //       // if (!preLoad) {
  //       //   fullPathCommand = fullPathCommand.andThen(
  //       //
  // AutoPathHelper.followPathWhileIntaking(AutoPathHelper.choreoCommand(firstTrajectory,
  //       // swerve, trajName),
  //       //           intake, Ground.LOWER_MOTION_SHOULDER_ANGLE,
  // Ground.LOWER_MOTION_WRIST_ANGLE));
  //       //   fullPath.remove(0);
  //       // }
  //       int i = 0;
  //       for (ChoreoTrajectory traj : fullPath) {
  //         Command trajCommand = AutoPathHelper.choreoCommand(traj, swerve, trajName);
  //         // fullPathCommand =
  //         // fullPathCommand.andThen(AutoPathHelper.doPathAndIntakeThenShoot(trajCommand,
  //         // swerve, shooter, intake, Ground.LOWER_MOTION_SHOULDER_ANGLE,
  //         // Ground.LOWER_MOTION_WRIST_ANGLE, roller));
  //         fullPathCommand =
  //             fullPathCommand.andThen(
  //                 AutoPathHelper.pathAndIntakeOnlyNew(
  //                     trajCommand, swerve, shooter, intake, roller, traj.getTotalTime(), i ==
  // 0));
  //         i++;
  //       }
  //     }
  //     fullPathCommand =
  //         fullPathCommand
  //             .beforeStarting(
  //                 Commands.parallel(
  //                         new AimTestCommand(
  //                             shooter,
  //                             () -> AllianceFlipUtil.apply(swerve.getPose()),
  //                             () -> swerve.getFieldRelativeSpeeds(),
  //                             roller,
  //                             true,
  //                             12,
  //                             true,
  //                             false,
  //                             false,
  //                             0),
  //                         new SequentialCommandGroup(
  //                             new WaitCommand(1.5),
  //                             Commands.run(() -> roller.setShooterFeederVoltage(11), roller)))
  //                     .withTimeout(1.7))
  //             .andThen(
  //                 Commands.parallel(
  //                     shooter.setShooterState(0, 0, 0).withTimeout(0.01),
  //                     new RollerCommand(roller, 0.0, false, () -> intake.shoulderGetRads())
  //                         .withTimeout(0.01))); // Last shot,
  //     // then
  //     // return to
  //     // neutral

  //     return fullPathCommand;
  //   }

  //   public Command buildAutonGrief(String trajName, boolean preLoad, double delay) {
  //     ArrayList<ChoreoTrajectory> fullPath = Choreo.getTrajectoryGroup(trajName);
  //     ChoreoTrajectory firstTrajectory =
  //         fullPath.size() > 0 ? fullPath.get(0) : new ChoreoTrajectory();
  //     ChoreoTrajectory lastTrajectory = fullPath.get(fullPath.size() - 1);
  //     // swerve.setYaw(firstTrajectory.getInitialPose().getRotation());
  //     Command fullPathCommand =
  //         Commands.runOnce(
  //                 () -> swerve.setPose(AllianceFlipUtil.apply(firstTrajectory.getInitialPose())))
  //             .andThen(
  //                 Commands.runOnce(
  //                     () ->
  //                         swerve.setTurnState(
  //                             swerve.kinematics.toSwerveModuleStates(
  //                                 fullPath.get(0).getInitialState().getChassisSpeeds())),
  //                     swerve));
  //     if (delay != 0.0) fullPathCommand = fullPathCommand.andThen(new WaitCommand(delay));
  //     if (fullPath.size() > 0) {
  //       if (!preLoad) {
  //         fullPathCommand =
  //             fullPathCommand.andThen(
  //                 AutoPathHelper.followPathWhileIntaking(
  //                     AutoPathHelper.choreoCommand(firstTrajectory, swerve, trajName),
  //                     intake,
  //                     Ground.LOWER_MOTION_SHOULDER_ANGLE,
  //                     Ground.LOWER_MOTION_WRIST_ANGLE));
  //         fullPath.remove(0);
  //       }
  //       for (ChoreoTrajectory traj : fullPath) {
  //         Command trajCommand = AutoPathHelper.choreoCommand(traj, swerve, trajName);
  //         // fullPathCommand =
  //         // fullPathCommand.andThen(AutoPathHelper.doPathAndIntakeThenShoot(trajCommand,
  //         // swerve, shooter, intake, Ground.LOWER_MOTION_SHOULDER_ANGLE,
  //         // Ground.LOWER_MOTION_WRIST_ANGLE, roller));
  //         fullPathCommand =
  //             fullPathCommand.andThen(
  //                 AutoPathHelper.doPathAndIntakeThenExtake(
  //                     trajCommand, swerve, shooter, intake, roller, traj.getTotalTime()));
  //       }
  //     }
  //     fullPathCommand =
  //         fullPathCommand.beforeStarting(
  //             Commands.parallel(
  //                     new AimTestCommand(
  //                             shooter,
  //                             () -> swerve.getPose(),
  //                             () -> swerve.getFieldRelativeSpeeds(),
  //                             roller,
  //                             true,
  //                             9.0,
  //                             true,
  //                             false,
  //                             false,
  //                             0)
  //                         // new SequentialCommandGroup(
  //                         // new WaitCommand(1),
  //                         // Commands.run(()-> roller.setShooterFeederVoltage(12), roller)
  //                         // ))
  //                         .withTimeout(2))
  //                 .andThen(
  //                     Commands.parallel(
  //                         shooter.setShooterState(0, 0, 0),
  //                         (new RollerCommand(roller, 0.0, false, () -> intake.shoulderGetRads()))
  //                             .withTimeout(0.01))));
  //     // swerve.setYaw(lastTrajectory.getFinalPose().getRotation());
  //     Logger.recordOutput("LAST TRAJECTORY ROTATION",
  // lastTrajectory.getFinalPose().getRotation());
  //     return fullPathCommand;
  //   }

  public SendableChooser<Command> buildAutoChooser() {
    SendableChooser<Command> out = new SendableChooser<Command>();
    out.setDefaultOption("1 Piece + Mobility Middle Subwoofer", oneMiddleSubwoofer(0.0));
    out.addOption("Just Leave", justLeave(0.0));
    out.addOption("Just Shoot Middle", justShoot(0.0, 0.0));
    out.addOption("Just Shoot Amp", justShoot(0.0, 60.0));
    out.addOption("Just Shoot Source", justShoot(0.0, -60.0));
    // out.addOption("1 Piece + Mobility Amp-side Subwoofer", oneAmpSide(0.0));
    // out.addOption("1 Piece + Mobility Enemy Source side Subwoofer", oneEnemySource(0.0));
    // out.addOption("2 Piece Middle Subwoofer", twoPieceCommand(0.0));
    // out.addOption("Source Side Griefer", sourceSideGrief());s
    // out.addOption("Source Side 1 Note", );

    // out.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     swerve.sysIdQuasistatic(Direction.kForward));
    // out.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     swerve.sysIdQuasistatic(Direction.kReverse));
    // out.addOption(
    //     "Drive SysId (Dynamic Forward)", swerve.sysIdDynamic(Direction.kForward));
    // out.addOption(
    //     "Drive SysId (Dynamic Reverse)", swerve.sysIdDynamic(Direction.kReverse));%
    // out.addOption("2 Note Speaker Side", "2 Note Speaker Side");
    // // out.addOption("3 Note Speaker Side", "3 Note Speaker Side");
    // out.addOption("4 Note Speaker", build4NoteAuton("4 Note Speaker", true, 0));
    // out.addOption("3 Note Source", buildAuton("3 Note Source", true, 0));
    // out.addOption("Slip Current Test", swerve.slipCurrentTest());

    // out.addOption("Source Side Grief", buildAuton("Source Side Grief", true, 0));

    // out.addOption("4 Note Amp", build4NoteAmpAuton("4 Note Amp", true, 0));
    // out.addOption("Grief Source", buildNewGriefAuton("Grief Source", true, 0));
    // out.addOption("2 Note Speaker Side", "2 Note Speaker Side");
    // out.addOption("DemoAutonPath", "DemoAutonPath");
    // out.addOption("4NoteStart", "4NoteStart");

    // out.addOption("BasicMovementChum", driveOnlyAuton("BasicMovementChum", 0));
    out.addOption("2025DriveTest", driveOnlyAuton("2025DriveTest", 0));
    out.addOption("2025TriggerTest", buildAutonWithTriggers("2025TriggerTest"));
    out.addOption("2025AutonTest", buildAuton("2025AutonTest", true, 0));
    out.addOption("FunnyPath", buildAuton("FunnyPath", true, 0));
    out.addOption("5 Note Speaker Side", buildAuton("5 Note Speaker Side", true, 0));
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

  public Command justLeave(double wait) {
    swerve.setYaw(new Rotation2d(0.0));
    return Commands.sequence(
            Commands.runOnce(
                () -> swerve.setPose(AllianceFlipUtil.apply(ShooterFlywheelConstants.subwoofer)),
                swerve),
            new WaitCommand(wait),
            Commands.run(() -> swerve.runVelocity(new ChassisSpeeds(2.0, 0.0, 0.0)), swerve)
                .withTimeout(1.25),
            Commands.run(() -> swerve.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0)), swerve)
                .withTimeout(1))
        .alongWith(shooter.setShooterState(0, 0, 0));
  }

  public Command oneMiddleSubwoofer(double wait) {
    swerve.setYaw(new Rotation2d(0.0));
    return Commands.sequence(
        Commands.runOnce(
            () -> swerve.setPose(AllianceFlipUtil.apply(ShooterFlywheelConstants.subwoofer)),
            swerve),
        new WaitCommand(wait),
        Commands.parallel(
                // new ShooterCommand(shooter, ()->
                // AllianceFlipUtil.apply(ShooterFlywheelConstants.subwoofer),()-> new
                // ChassisSpeeds(0.0,0.0,0.0), roller, false, 9.0),
                aimPresetGyroStationaryAuto(
                    AllianceFlipUtil.apply(ShooterFlywheelConstants.subwoofer), 9),
                Commands.sequence(
                    new WaitCommand(2),
                    Commands.run(() -> roller.setShooterFeederVoltage(12), roller)))
            .withTimeout(3),
        Commands.race(
            shooter.setShooterState(0, 0, 0),
            (new RollerCommand(roller, 0.0, false, () -> intake.shoulderGetRads())),
            Commands.run(() -> swerve.runVelocity(new ChassisSpeeds(2.0, 0.0, 0.0)), swerve)
                .withTimeout(1)),
        Commands.run(() -> swerve.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0)), swerve));
  }

  public Command justShoot(double wait, double angle) {
    return Commands.sequence(
        Commands.runOnce(
            () ->
                swerve.setPose(
                    AllianceFlipUtil.apply(
                        new Pose2d(
                            ShooterFlywheelConstants.subwoofer.getTranslation(),
                            AllianceFlipUtil.apply(
                                new Rotation2d(Units.degreesToRadians(angle)))))),
            swerve),
        new WaitCommand(wait),
        Commands.parallel(
                // new ShooterCommand(shooter, ()->
                // AllianceFlipUtil.apply(ShooterFlywheelConstants.subwoofer),()-> new
                // ChassisSpeeds(0.0,0.0,0.0), roller, false, 9.0),
                aimPresetGyroStationaryAuto(
                    AllianceFlipUtil.apply(ShooterFlywheelConstants.subwoofer), 9),
                Commands.sequence(
                    new WaitCommand(2),
                    Commands.run(() -> roller.setShooterFeederVoltage(12), roller)))
            .withTimeout(3),
        Commands.parallel(
            shooter.setShooterState(0, 0, 0),
            (new RollerCommand(roller, 0.0, false, () -> intake.shoulderGetRads()))
                .withTimeout(0.01)));
  }
}
