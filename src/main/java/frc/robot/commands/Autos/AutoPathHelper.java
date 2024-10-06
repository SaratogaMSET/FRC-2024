package frc.robot.commands.Autos;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Intake.DesiredStates.Ground;
import frc.robot.commands.Intake.IntakePositionCommand;
import frc.robot.commands.Intake.RollerCommand;
import frc.robot.commands.Intake.RollerCommandExtake;
import frc.robot.commands.Intake.RollerToShooterIR;
import frc.robot.commands.Intake.RollerToShooterIRWithoutShooter;
import frc.robot.commands.Shooter.AimTestCommand;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.Roller.RollerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.util.AllianceFlipUtil;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AutoPathHelper {

  // Might save loc?
  //  Command createAmpToBounceDEF() {
  //     var firstSegment = loadSegment(Location.AMP, Location.PREPLACE_D);

  //     preloadTrajectoryClass(firstSegment);

  //     SequentialCommandGroup c = new SequentialCommandGroup();

  //     c.addCommands(resetPose(firstSegment));
  //     c.addCommands(follow(firstSegment)
  //             .alongWith(shootPreloadWithHardcodedPivotAndTurret(
  //                     Rotation2d.fromDegrees(180.0), Rotation2d.fromDegrees(45.0))));
  //     c.addCommands(followThenShoot(Location.PREPLACE_D, Location.AMP_SHOT));
  //     c.addCommands(followWhileIntaking(Location.AMP_SHOT, Location.PREPLACE_E));
  //     c.addCommands(followThenShoot(Location.PREPLACE_E, Location.AMP_SHOT));
  //     c.addCommands(followWhileIntaking(Location.AMP_SHOT, Location.PREPLACE_F));
  //     c.addCommands(followThenShoot(Location.PREPLACE_F, Location.STAGE_SHOT));
  //     c.addCommands(followWhileIntaking(Location.STAGE_SHOT, Location.PREPLACE_E));
  //     return c;
  // }

  /*
  /* also the obvious but make sure ur choreo current limit is sane and you have at least 10a of stator limit headroom on your motors above choreo
  */

  //   static HolonomicPathFollowerConfig config =
  //       new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live
  // in
  //           // your Constants class
  //           new PIDConstants(7.5, 0.0, 0.0), // Translation PID constants
  //           new PIDConstants(7.5, 0.0, 0.0), // Rotation PID constants
  //           4.5, // Max module speed, in m/s
  //           0.4, // Drive base radius in meters. Distance from robot center to furthest module.
  //           new ReplanningConfig()); // Default path replanning config. See the API for the
  // options
  //   // here
  //   public static PathConstraints constraints = new PathConstraints(4.5, 3, 3, 2);

    public static Command intakeCommand(IntakeSubsystem intake, RollerSubsystem roller, ShooterSubsystem shooter){
            return new IntakePositionCommand(
                    intake, Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE)
                .asProxy()
                .alongWith(
                    new RollerToShooterIR(roller, shooter, 9.0)); // DID NOT HAVE .asProxy() before
    }

    public static Command intakeCommandWithoutShooter(IntakeSubsystem intake, RollerSubsystem roller) {
            return new IntakePositionCommand(
                    intake, Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE)
                .asProxy()
                .alongWith(
                    new RollerToShooterIRWithoutShooter(roller, 9.0)); // DID NOT HAVE .asProxy() before
    }

  public static Command followPath(String pathToFollow) {
    return AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(pathToFollow))
        .withName(pathToFollow);
  }

  public static Command followPathWhileIntaking(
      Command path, IntakeSubsystem intake, double shoulderAngle, double wristAngle) {
    return Commands.deadline(path, new IntakePositionCommand(intake, shoulderAngle, wristAngle));
  }

  public static Command followPathWhileShooting(
      String pathToFollow,
      ShooterSubsystem shooterSubsystem,
      SwerveSubsystem swerve,
      RollerSubsystem roller) {
    return followPath(pathToFollow)
        .alongWith(
            new AimTestCommand(
                shooterSubsystem,
                () -> swerve.getPose(),
                () -> new ChassisSpeeds(0.0, 0.0, 0.0),
                roller,
                true,
                7.6,
                true,
                false,
                false,
                0))
        .withName(pathToFollow);
  }

  public static Command followPathAfterShooting(
      String pathToFollow,
      ShooterSubsystem shooterSubsystem,
      SwerveSubsystem swerve,
      RollerSubsystem roller) {
    return followPath(pathToFollow)
        .beforeStarting(
            new AimTestCommand(
                shooterSubsystem,
                () -> swerve.getPose(),
                () -> new ChassisSpeeds(0.0, 0.0, 0.0),
                roller,
                true,
                7.6,
                true,
                false,
                false,
                0))
        .withName(pathToFollow);
  }

  /**
   * Shoot with Aim-Test-Command, then move along the choreo path whilst intaking until it reaches
   * the shooter-ir
   *
   * <p>Initial shot: Aim-Test-Command -> Rev For 1.5 Seconds, then Shoot.
   *
   * <p>Intaking Command : Move Intake down to ground position and spin rollers in parallel, until
   * it arrives at the roller beam break.
   *
   * <p>Intaking Command runs in race with path command, Either the path finishes(+ 1 second) first
   * or we intake the note and the path finishes.
   *
   * @param path
   * @param swerve
   * @param shooter
   * @param intake
   * @param roller
   * @param time
   * @return
   */
  public static Command shootThenPathAndIntake(
      Command path,
      SwerveSubsystem swerve,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      RollerSubsystem roller,
      double time) {
    // Misnomer, should shoot before pathing and intaking :D

    SequentialCommandGroup shot = new SequentialCommandGroup();

    shot.addCommands(
        Commands.parallel(
                new AimTestCommand(
                    shooter,
                    () -> swerve.getPose(),
                    () -> new ChassisSpeeds(0.0, 0.0, 0.0),
                    roller,
                    true,
                    11,
                    true,
                    false,
                    false,
                    0),
                Commands.sequence(
                    new WaitCommand(1),
                    Commands.run(() -> roller.setShooterFeederVoltage(12), roller)))
            .withTimeout(1.2));
    shot.addCommands(
        Commands.parallel(
                new AimTestCommand(
                    shooter,
                    () -> swerve.getPose(),
                    () -> new ChassisSpeeds(0.0, 0.0, 0.0),
                    roller,
                    true,
                    11,
                    true,
                    false,
                    false,
                    0),
                Commands.sequence(
                    new WaitCommand(1),
                    Commands.run(() -> roller.setShooterFeederVoltage(12), roller)))
            .withTimeout(1.2));
    shot.addCommands(
        Commands.parallel(
            shooter.setShooterState(0, 0, 0).withTimeout(0.01),
            new RollerCommand(roller, 0.0, false, () -> intake.shoulderGetRads())
                .withTimeout(0.01)));
    shot.addCommands(
        Commands.deadline(
            path.andThen(new WaitCommand(0.9)), // Command ends upon path time completion
            intakeCommand(intake, roller, shooter)));
    return shot;
  }

  /**
   * Intake the note and rev, then shoot. Then path whilst intaking.
   *
   * <p>Decreases rev time by assuming shots don't require turret/pivot movement.
   *
   * @param path
   * @param swerve
   * @param shooter
   * @param intake
   * @param roller
   * @param time
   * @return
   */
  public static Command pathAndIntakeThenIntakeRevThenShot(
      Command path,
      SwerveSubsystem swerve,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      RollerSubsystem roller,
      double time) {

    SequentialCommandGroup shot = new SequentialCommandGroup();

    shot.addCommands(
        Commands.deadline(
            path, // .andThen(new WaitCommand(0.9)), // Command ends upon path time completion
            intakeCommand(intake, roller, shooter)));
    shot.addCommands(
        Commands.sequence(
            Commands.parallel(
                    shooter.setShooterStateMPS(9, 0, 44),
                    // new AimTestCommand(shooter, ()-> swerve.getPose(), ()-> new
                    // ChassisSpeeds(0.0,0.0,0.0), roller, true, 11, true, false, false, 0),
                    intakeCommandWithoutShooter(intake, roller))
                .withTimeout(0.8),
            Commands.parallel(
                    new AimTestCommand(
                        shooter,
                        () -> swerve.getPose(),
                        () -> new ChassisSpeeds(0.0, 0.0, 0.0),
                        roller,
                        true,
                        11,
                        true,
                        false,
                        false,
                        0),
                    Commands.sequence(
                        new WaitCommand(1),
                        Commands.run(() -> roller.setShooterFeederVoltage(12), roller)))
                .withTimeout(1.2) // 0.2 seconds per shot
            ));
    shot.addCommands(
        Commands.parallel(
            shooter.setShooterState(0, 0, 0).withTimeout(0.01),
            new RollerCommand(roller, 0.0, false, () -> intake.shoulderGetRads())
                .withTimeout(0.01)));

    return shot;
  }
  /**
   * Intake the note and rev, then shoot. Then path whilst intaking.
   *
   * <p>Decreases rev time by assuming shots don't require turret/pivot movement.
   *
   * @param path
   * @param swerve
   * @param shooter
   * @param intake
   * @param roller
   * @param time
   * @return
   */
  public static Command pathAndIntakeThenIntakeRevThenShotFor4Note(
      Command path,
      SwerveSubsystem swerve,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      RollerSubsystem roller,
      double time,
      Pose2d suppliedPose) {
    // Misnomer, should shoot before pathing and intaking :D
    SequentialCommandGroup shot = new SequentialCommandGroup();

    shot.addCommands(
        Commands.deadline(
            path.andThen(new WaitCommand(0.9)), // Command ends upon path time completion
            intakeCommand(intake, roller, shooter)));
    shot.addCommands(
        Commands.sequence(
            Commands.parallel(
                    new AimTestCommand(
                        shooter,
                        () -> swerve.getPose(),
                        () -> new ChassisSpeeds(0.0, 0.0, 0.0),
                        roller,
                        true,
                        9,
                        true,
                        false,
                        false,
                        0),
                    Commands.sequence(
                        new WaitCommand(1),
                        Commands.run(() -> roller.setShooterFeederVoltage(12), roller)))
                .withTimeout(1.2) // 0.2 seconds per shot
            ));
    shot.addCommands(
        Commands.parallel(
            shooter.setShooterState(0, 0, 0).withTimeout(0.01),
            new RollerCommand(roller, 0.0, false, () -> intake.shoulderGetRads())
                .withTimeout(0.01)));
    return shot;
  }

  public static Command pathAndIntakeThenIntakeRevThenShotFor4NoteAmp(
      Command path,
      SwerveSubsystem swerve,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      RollerSubsystem roller,
      double time) {
    // Misnomer, should shoot before pathing and intaking :D

    SequentialCommandGroup shot = new SequentialCommandGroup();

    shot.addCommands(
            Commands.deadline(
                path.andThen(new WaitCommand(0.4)), // Command ends upon path time completion
                intakeCommand(intake, roller, shooter)));
    shot.addCommands(
            Commands.sequence(
                Commands.parallel(
                        new AimTestCommand(
                            shooter,
                            () -> swerve.getPose(),
                            () -> new ChassisSpeeds(0.0, 0.0, 0.0),
                            roller,
                            true,
                            12,
                            true,
                            false,
                            false,
                            0),
                        Commands.sequence(
                            new WaitCommand(1.4),
                            Commands.run(() -> roller.setShooterFeederVoltage(12), roller)))
                    .withTimeout(1.6) // 0.2 seconds per shot
                ));
        shot.addCommands(
            Commands.parallel(
                shooter.setShooterState(0, 0, 0).withTimeout(0.01),
                new RollerCommand(roller, 0.0, false, () -> intake.shoulderGetRads())
                    .withTimeout(0.01)));

    Command out =
        shot.beforeStarting(shooter.shooterVoltage(0, 0).withTimeout(0))
            .andThen(shooter.shooterVoltage(0, 0).withTimeout(0));

    return out;
  }

  public static Command pathAndIntakeOnlyNew(
      Command path,
      SwerveSubsystem swerve,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      RollerSubsystem roller,
      double time,
      boolean intaking) {
    // Misnomer, should shoot before pathing and intaking :D
    Command shot = null;

    if (intaking) {
      shot =
          Commands.sequence(
                  Commands.deadline(
                      path.andThen(new WaitCommand(0.4)), // Command ends upon path time completion
                      intakeCommand(intake, roller, shooter)))
              .beforeStarting(shooter.shooterVoltage(0, 0).withTimeout(0))
              .andThen(shooter.shooterVoltage(0, 0).withTimeout(0));
    } else {
      return path;
    }

    Command out = shot;

    return out;
  }
  /**
   * Shoot with Aim-Test-Command, then move along the choreo path whilst intaking until it reaches
   * the shooter-ir
   *
   * <p>Initial shot: Aim-Test-Command -> Rev For 1.5 Seconds, then Shoot.
   *
   * <p>Intaking Command : Move Intake down to ground position and spin rollers in parallel, until
   * it arrives at the roller beam break.
   *
   * <p>Intaking Command runs in race with path command, Either the path finishes(+ 1 second) first
   * or we intake the note and the path finishes.
   *
   * @param path
   * @param swerve
   * @param shooter
   * @param intake
   * @param roller
   * @param time
   * @return
   */
  public static Command shootThenPathAndIntakeFor3NoteSource(
      Command path,
      SwerveSubsystem swerve,
      ShooterSubsystem shooter,
      IntakeSubsystem intake,
      RollerSubsystem roller,
      double time) {

    Command shot = null;

    shot =
        Commands.sequence(
            Commands.parallel(
                    new AimTestCommand(
                        shooter,
                        () -> swerve.getPose(),
                        () -> new ChassisSpeeds(0.0, 0.0, 0.0),
                        roller,
                        true,
                        11,
                        true,
                        false,
                        false,
                        0),
                    Commands.sequence(
                        new WaitCommand(1),
                        Commands.run(() -> roller.setShooterFeederVoltage(12), roller)))
                .withTimeout(1.2),
            Commands.parallel(
                shooter.setShooterState(0, 0, 0).withTimeout(0.01),
                new RollerCommand(roller, 0.0, false, () -> intake.shoulderGetRads())
                    .withTimeout(0.01)),
            Commands.deadline(
                path, // Command ends upon path time completion
                intakeCommand(intake, roller, shooter)));

    Command out = shot;

    return out;
  }

  public static Command doPathAndIntakeThenExtake(
      Command path,
      SwerveSubsystem swerve,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intake,
      RollerSubsystem roller,
      double time) {
    Command intakeCommand =
        new IntakePositionCommand(
                intake, Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE)
            .alongWith(
                new RollerCommandExtake(roller, 10, false, () -> intake.shoulderGetRads())
                    .alongWith(shooterSubsystem.anglingDegrees(0.0, 44)))
        ;

    Command out = Commands.race(path, intakeCommand); // withTimeout(time);

    return out;
  }

  // what on earth
  public static Command doShootThenIntakeThenExtake(
      Command path,
      SwerveSubsystem swerve,
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intake,
      RollerSubsystem roller,
      double time) {
    Command intakeCommand =
        new IntakePositionCommand(
                intake, Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE)
            .alongWith(
                new RollerCommand(roller, 10, false, () -> intake.shoulderGetRads())
                    .alongWith(shooterSubsystem.anglingDegrees(0.0, 44)))
            .andThen(
                Commands.run(() -> roller.setShooterFeederVoltage(0.9), roller)
                    .until(() -> roller.getShooterBeamBreak()));

    Command out = Commands.race(path, intakeCommand);
    out.beforeStarting(
        Commands.parallel(
                new AimTestCommand(
                    shooterSubsystem,
                    () -> swerve.getPose(),
                    () -> new ChassisSpeeds(0.0, 0.0, 0.0),
                    roller,
                    true,
                    7.6,
                    true,
                    false,
                    false,
                    0)
                // Commands.sequence(
                //     new WaitCommand(2),
                //     Commands.run(()-> roller.setShooterFeederVoltage(12), roller) )
                )
            .withTimeout(3)); // withTimeout(time);

    return out;
  }

  public static Command sequencePaths(SwerveSubsystem swerve, Command... paths) {
    return Commands.runOnce(
            () ->
                swerve.setPose(
                    AllianceFlipUtil.apply(
                        PathPlannerPath.fromChoreoTrajectory(paths[0].getName())
                            .getPreviewStartingHolonomicPose())),
            swerve)
        .andThen(paths);
  }

  public static Command choreoCommand(
      ChoreoTrajectory traj, SwerveSubsystem swerve, String trajName) {
    var thetaController = new PIDController(1, 0.0, 0); // 1
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    Command swerveCommand =
        choreoFullFollowSwerveCommand(
            traj,
            swerve::getPose, // swerve::getPoseGyroRot
            Choreo.choreoSwerveController(
                new PIDController(1, 0.0, 0), //  TODO:  5 maybe
                new PIDController(1.0, 0.0, 0), // 1
                thetaController),
            (ChassisSpeeds speeds) -> swerve.runVelocity(speeds),
            () ->
                DriverStation.getAlliance().isPresent()
                    && DriverStation.getAlliance().get() == Alliance.Red,
            Units.inchesToMeters(4),
            Units.inchesToMeters(4),
            3,
            swerve);
    return swerveCommand;
  }

  // TODO: Maybe add in later! OTF Generation
  //   public static Command pathfindToPose(Pose2d targetPose, SwerveSubsystem swerveSubsystem) {
  //     return AutoBuilder.pathfindToPose(targetPose, constraints);
  //   }

  //   public static Command pathfindToPath(PathPlannerPath path) {
  //     return AutoBuilder.pathfindThenFollowPath(path, constraints);
  //   }

  /**
   * Create a command to follow a Choreo path.
   *
   * @param trajectory The trajectory to follow. Use Choreo.getTrajectory(String trajName) to load
   *     this from the deploy directory.
   * @param poseSupplier A function that returns the current field-relative pose of the robot.
   * @param controller A ChoreoControlFunction to follow the current trajectory state. Use
   *     ChoreoCommands.choreoSwerveController(PIDController xController, PIDController yController,
   *     PIDController rotationController) to create one using PID controllers for each degree of
   *     freedom. You can also pass in a function with the signature (Pose2d currentPose,
   *     ChoreoTrajectoryState referenceState) -&gt; ChassisSpeeds to implement a custom follower
   *     (i.e. for logging).
   * @param outputChassisSpeeds A function that consumes the target robot-relative chassis speeds
   *     and commands them to the robot.
   * @param mirrorTrajectory If this returns true, the path will be mirrored to the opposite side,
   *     while keeping the same coordinate system origin. This will be called every loop during the
   *     command.
   * @param requirements The subsystem(s) to require, typically your drive subsystem only.
   * @return A command that follows a Choreo path.
   */
  public static Command choreoFullFollowSwerveCommand(
      ChoreoTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      ChoreoControlFunction controller,
      Consumer<ChassisSpeeds> outputChassisSpeeds,
      BooleanSupplier mirrorTrajectory,
      double toleranceX,
      double toleranceY,
      double toleranceTheta,
      Subsystem... requirements) {
    var timer = new Timer();
    return new FunctionalCommand(
        () -> {
          timer.restart();
          Logger.recordOutput(
              "Choreo/Active Traj",
              (mirrorTrajectory.getAsBoolean() ? trajectory.flipped() : trajectory).getPoses());
        },
        () -> {
          Logger.recordOutput(
              "Choreo/Target Pose",
              trajectory.sample(timer.get(), mirrorTrajectory.getAsBoolean()).getPose());
          Logger.recordOutput(
              "Choreo/Target Speeds",
              trajectory.sample(timer.get(), mirrorTrajectory.getAsBoolean()).getChassisSpeeds());
          outputChassisSpeeds.accept(
              controller.apply(
                  poseSupplier.get(),
                  trajectory.sample(timer.get(), mirrorTrajectory.getAsBoolean())));
        },
        (interrupted) -> {
          timer.stop();
          if (interrupted) {
            outputChassisSpeeds.accept(new ChassisSpeeds());
          } else {
            outputChassisSpeeds.accept(trajectory.getFinalState().getChassisSpeeds());
          }
        },
        () -> {
          var finalPose =
              mirrorTrajectory.getAsBoolean()
                  ? trajectory.getFinalState().flipped().getPose()
                  : trajectory.getFinalState().getPose();
          Logger.recordOutput("Swerve/Current Traj End Pose", finalPose);
          return timer.hasElapsed(trajectory.getTotalTime())
              || (MathUtil.isNear(finalPose.getX(), poseSupplier.get().getX(), toleranceX)
                  && MathUtil.isNear(finalPose.getY(), poseSupplier.get().getY(), toleranceY)
                  && Math.abs(
                          (poseSupplier.get().getRotation().getDegrees()
                                  - finalPose.getRotation().getDegrees())
                              % 360)
                      < toleranceTheta);
        },
        requirements);
  }
}
