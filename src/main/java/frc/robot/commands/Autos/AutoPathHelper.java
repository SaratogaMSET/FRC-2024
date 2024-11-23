package frc.robot.commands.Autos;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoFactory.AutoBindings;
import choreo.auto.AutoLoop;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Intake.DesiredStates.Ground;
import frc.robot.RobotContainer;
import frc.robot.commands.Intake.IntakePositionCommand;
import frc.robot.commands.Intake.RollerCommand;
import frc.robot.commands.Intake.RollerToShooterIR;
import frc.robot.commands.Intake.RollerToShooterIRWithoutShooter;
import frc.robot.commands.Shooter.AimTestCommand;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.Roller.RollerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.util.AllianceFlipUtil;
import java.util.Arrays;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class AutoPathHelper {
  private static AutoFactory autoFactory;

  public static void createFactory(SwerveSubsystem swerve) {
    if (autoFactory == null) {
      autoFactory =
          Choreo.createAutoFactory(
              swerve,
              swerve::getPose,
              swerve::choreoController,
              () ->
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red,
              new AutoBindings(),
              AutoPathHelper::trajLogger);
    }
  }

  public static AutoFactory getFactory() {
    return autoFactory;
  }

  public static Command aimAutonomous() {
    return new WaitCommand(0);
  }

  public static Command shootAutonomous(
      SwerveSubsystem swerve, ShooterSubsystem shooter, RollerSubsystem roller) {
    return Commands.parallel(
            RobotContainer.aimRobotGyroStationaryAuto(11, swerve, shooter),
            new SequentialCommandGroup(
                Commands.waitUntil(() -> shooter.shooterReady()),
                Commands.run(() -> roller.setShooterFeederVoltage(11), roller).withTimeout(1.5)))
        .withTimeout(1.5);
  }

  public static Command intakeCommand(
      IntakeSubsystem intake, RollerSubsystem roller, ShooterSubsystem shooter) {
    return new IntakePositionCommand(
            intake, Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE)
        .asProxy()
        .alongWith(new RollerToShooterIR(roller, shooter, 9.0)); // DID NOT HAVE .asProxy() before
  }

  public static Command intakeCommandWithoutShooter(
      IntakeSubsystem intake, RollerSubsystem roller) {
    return new IntakePositionCommand(
            intake, Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE)
        .asProxy()
        .alongWith(
            new RollerToShooterIRWithoutShooter(roller, 9.0)); // DID NOT HAVE .asProxy() before
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
    return choreoCommand(swerve, pathToFollow)
        .alongWith(shootAutonomous(swerve, shooterSubsystem, roller))
        .withName(pathToFollow);
  }

  public static Command followPathAfterShooting(
      String pathToFollow,
      ShooterSubsystem shooterSubsystem,
      SwerveSubsystem swerve,
      RollerSubsystem roller) {
    return choreoCommand(swerve, pathToFollow)
        .beforeStarting(shootAutonomous(swerve, shooterSubsystem, roller))
        .withName(pathToFollow);
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
                    intakeCommandWithoutShooter(intake, roller))
                .withTimeout(0.8),
            Commands.parallel(
                    new AimTestCommand(
                        shooter,
                        () -> swerve.getPose(),
                        () -> new ChassisSpeeds(0.0, 0.0, 0.0),
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

  public static Command shot(
      SwerveSubsystem swerve,
      ShooterSubsystem shooter,
      RollerSubsystem roller,
      IntakeSubsystem intake) {
    return Commands.sequence(
        new InstantCommand(() -> swerve.stop()),
        new ConditionalCommand(
            shootAutonomous(swerve, shooter, roller),
            new WaitCommand(0),
            () -> roller.getShooterBeamBreak()),
        Commands.parallel(
            shooter.setShooterState(0, 0, 0).withTimeout(0.01),
            new RollerCommand(roller, 0.0, false, () -> intake.shoulderGetRads())
                .withTimeout(0.01)));
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
      RollerSubsystem roller) {

    Command shot = null;

    shot =
        Commands.sequence(
            new InstantCommand(() -> swerve.stop()),
            new ConditionalCommand(
                shootAutonomous(swerve, shooter, roller),
                new WaitCommand(0),
                () -> roller.getShooterBeamBreak()),
            Commands.parallel(
                shooter.setShooterState(0, 0, 0).withTimeout(0.01),
                new RollerCommand(roller, 0.0, false, () -> intake.shoulderGetRads())
                    .withTimeout(0.01)),
            Commands.deadline(
                path.andThen(
                    new InstantCommand(() -> swerve.stop())
                        .andThen(
                            Commands.waitUntil(() -> roller.getShooterBeamBreak())
                                .withTimeout(1))), // Command ends upon path time completion
                intakeCommand(intake, roller, shooter)));

    Command out = shot;

    return out;
  }

  public static Command choreoCommand(SwerveSubsystem swerve, String trajName) {
    return autoFactory.trajectoryCommand(trajName);
  }

  public static Command choreoCommand(SwerveSubsystem swerve, String trajName, int split) {
    return autoFactory.trajectoryCommand(trajName, split);
  }

  public static Command choreoAutoWithTriggers(
      SwerveSubsystem swerve,
      IntakeSubsystem intake,
      RollerSubsystem roller,
      ShooterSubsystem shooter,
      String trajName) {

    final AutoLoop loop = autoFactory.newLoop(trajName);
    int numberOfSplits = Choreo.loadTrajectory(trajName).get().splits().size(); //How many trajectories we have
    AutoTrajectory[] trajectories = new AutoTrajectory[numberOfSplits];
    for (int i = numberOfSplits - 1; i >= 0; i--) { //Loop through splits backwards
      trajectories[i] = autoFactory.trajectory(trajName, i, loop);
      AutoTrajectory trajectory = trajectories[i];
      trajectory
          .atTime("Intake")
          .onTrue(intakeCommand(intake, roller, shooter)); //Intake when reached Event Marker in Choreo
      //If this isn't the last trajectory, set the end of it to shoot and trigger the next trajectory, otherwise just shoot
      if (i != numberOfSplits - 1){ 
        trajectory
            .done()
            .onTrue(shot(swerve, shooter, roller, intake).andThen(trajectories[i + 1].cmd()));
      } else {
        trajectory
            .done()
            .onTrue(shot(swerve, shooter, roller, intake));
      }
    }
    //Set odom
    loop.enabled()
        .onTrue(new InstantCommand(() -> swerve.setPose(trajectories[0].getInitialPose().get())));
    //Start first trajectory
    loop.enabled().onTrue(trajectories[0].cmd());

    // trajectory
    //     .atTime("Shoot")
    //     .onTrue(
    //         new SequentialCommandGroup(
    //             new InstantCommand(() -> swerve.stop()),
    //             new ConditionalCommand(
    //                 shootAutonomous(swerve, shooter, roller),
    //                 new WaitCommand(5).andThen(new InstantCommand( () ->
    // Logger.recordOutput("runningAutoShooter", true))),
    //                 () -> roller.getShooterBeamBreak()),
    //             Commands.parallel(
    //                 shooter.setShooterState(0, 0, 0).withTimeout(0.01),
    //                 new RollerCommand(roller, 0.0, false, () -> intake.shoulderGetRads())
    //                     .withTimeout(0.01))
    //         )
    //       );
    return loop.cmd();
  }

  public static Pose2d initialPose(Optional<Trajectory<SwerveSample>> fullPath) {
    return fullPath
        .get()
        .getInitialPose(
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red);
  }

  public static void trajLogger(Trajectory<SwerveSample> trajectory, Boolean atEndpoints) {
    Logger.recordOutput(
        "Choreo/Active Traj",
        Arrays.stream(trajectory.sampleArray())
            .map((SwerveSample s) -> AllianceFlipUtil.apply(s.getPose()))
            .toArray(Pose2d[]::new));

    // Logger.recordOutput("Choreo/Target Pose", trajectory.sampleAt(trajectory.time, false));
  }
}
