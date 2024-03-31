package frc.robot.commands.Autos;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoControlFunction;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Intake.DesiredStates.Ground;
import frc.robot.Robot;
import frc.robot.commands.Intake.IntakeNeutralCommand;
import frc.robot.commands.Intake.IntakePositionCommand;
import frc.robot.commands.Intake.RollerCommand;
import frc.robot.commands.Intake.RollerCommandExtake;
import frc.robot.commands.Shooter.AimTestCommand;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.Roller.RollerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.util.AllianceFlipUtil;
import static frc.robot.subsystems.Swerve.Module.WHEEL_RADIUS;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;


public class AutoPathHelper {


    static HolonomicPathFollowerConfig config = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(7.5, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(7.5, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig()); // Default path replanning config. See the API for the options here
    public static PathConstraints constraints = new PathConstraints(
        4.5, 3,
        3, 2);
    public static Command annotateName(Command x, String pathToFollow){
        x.setName(pathToFollow);
        return x;
    }

    public static Command followPath(String pathToFollow){
        return annotateName(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(pathToFollow)), pathToFollow);
    }
    public static Command followPathWhileIntaking(Command path, IntakeSubsystem intake, double shoulderAngle, double wristAngle){
        return Commands.deadline(path, new IntakePositionCommand(intake, shoulderAngle, wristAngle));

    } 

     public static Command followPathWhileShooting(String pathToFollow, ShooterSubsystem shooterSubsystem, SwerveSubsystem swerve, RollerSubsystem roller){
        return annotateName(followPath(pathToFollow)
        .alongWith(new AimTestCommand(shooterSubsystem, ()->swerve.getPose() , ()-> swerve.getFieldRelativeSpeeds(), roller, true, 7.6, true, false, false)), pathToFollow);
    }
     public static Command followPathAfterShooting(String pathToFollow, ShooterSubsystem shooterSubsystem, SwerveSubsystem swerve, RollerSubsystem roller){
        return annotateName(followPath(pathToFollow)
        .beforeStarting(new AimTestCommand(shooterSubsystem, ()-> swerve.getPose(), ()-> swerve.getFieldRelativeSpeeds(), roller, true, 7.6, true, false, false)), pathToFollow);
    }
    public static Command doPathAndIntakeThenShoot(Command path, SwerveSubsystem swerve, ShooterSubsystem shooterSubsystem, IntakeSubsystem intake, RollerSubsystem roller, double time) {
        Command intakeCommand =
            new IntakePositionCommand(intake, Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE)
            .alongWith(
                new RollerCommand(roller, 10, false, ()->intake.shoulderGetRads()).alongWith(shooterSubsystem.anglingDegrees(0.0,44)))
            .andThen(Commands.run(()->roller.setShooterFeederVoltage(0.9), roller).until(()->roller.getShooterBeamBreak()))
            .andThen(() -> {System.out.println("Intake Command Finished");});

        Command out = Commands.race(path.andThen(new WaitCommand(1)).andThen(() -> {System.out.println("Path Command Finished");}), intakeCommand); //withTimeout(time);

        return out.beforeStarting(
            Commands.parallel(
                new AimTestCommand(shooterSubsystem, ()-> swerve.getPose(), ()-> new ChassisSpeeds(0.0,0.0,0.0), roller, true, 7.6, true, false, false)
                // Commands.sequence(
                //     new WaitCommand(2),
                //     Commands.run(()-> roller.setShooterFeederVoltage(12), roller) )
                ).withTimeout(3));        
    }
    public static Command doPathAndIntakeThenExtake(Command path, SwerveSubsystem swerve, ShooterSubsystem shooterSubsystem, IntakeSubsystem intake, RollerSubsystem roller, double time) {
        Command intakeCommand =
            new IntakePositionCommand(intake, Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE)
            .alongWith(
                new RollerCommandExtake(roller, 10, false, ()->intake.shoulderGetRads()).alongWith(shooterSubsystem.anglingDegrees(0.0,44)))
            .andThen(() -> {System.out.println("Intake Command Finished");});

        Command out = Commands.race(path.andThen(() -> {System.out.println("Path Command Finished");}), intakeCommand); //withTimeout(time);

        return out;
                // Commands.sequence(
                //     new WaitCommand(2),
                //     Commands.run(()-> roller.setShooterFeederVoltage(12), roller) ) 
    }
    public static Command doShootThenIntakeThenExtake(Command path, SwerveSubsystem swerve, ShooterSubsystem shooterSubsystem, IntakeSubsystem intake, RollerSubsystem roller, double time) {
        Command intakeCommand =
            new IntakePositionCommand(intake, Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE)
            .alongWith(
                new RollerCommand(roller, 10, false, ()->intake.shoulderGetRads()).alongWith(shooterSubsystem.anglingDegrees(0.0,44)))
            .andThen(Commands.run(()->roller.setShooterFeederVoltage(0.9), roller).until(()->roller.getShooterBeamBreak()))
            .andThen(() -> {System.out.println("Intake Command Finished");});

        Command out = Commands.race(path.andThen(() -> {System.out.println("Path Command Finished");}), intakeCommand);
        out.beforeStarting(
            Commands.parallel(
                new AimTestCommand(shooterSubsystem, ()-> swerve.getPose(), ()-> new ChassisSpeeds(0.0,0.0,0.0), roller, true, 7.6, true, false, false)
                // Commands.sequence(
                //     new WaitCommand(2),
                //     Commands.run(()-> roller.setShooterFeederVoltage(12), roller) )
                ).withTimeout(3)); //withTimeout(time);
     
        return out;
    }
    public static Command sequencePaths(SwerveSubsystem swerve, Command... paths) {
        return Commands.runOnce(()-> swerve.setPose(AllianceFlipUtil.apply(PathPlannerPath.fromChoreoTrajectory(paths[0].getName()).getPreviewStartingHolonomicPose())), swerve).andThen(paths);
    }
    public static Command choreoCommand(ChoreoTrajectory traj, SwerveSubsystem swerve, String trajName) {
        var thetaController = new PIDController(1.5, 0.0, 0.5); //1
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    //     Command swerveCommand = Choreo.choreoSwerveCommand(
    //     traj,
    //     swerve::getPose,
    //     new PIDController(1, 0.0, 0.0), //7
    //     new PIDController(1, 0.0, 0.0), //7
    //     thetaController,
    //     (ChassisSpeeds speeds) -> swerve.runVelocity(speeds),
    //     () -> DriverStation.getAlliance().isPresent()
    //             && DriverStation.getAlliance().get() == Alliance.Red,
    //     swerve
    // );
    Command swerveCommand = choreoFullFollowSwerveCommand(
        traj,
        swerve::getPose,
        Choreo.choreoSwerveController(
            new PIDController(4, 0.0, 0.5), //7
            new PIDController(4, 0.0, 0.5), //7
            thetaController),
        (ChassisSpeeds speeds) -> swerve.runVelocity(speeds),
        () -> DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        Units.inchesToMeters(7),
        Units.inchesToMeters(7),
        3,
        swerve
    );
    return swerveCommand;
    }

    public static Command pathfindToPose(Pose2d targetPose, SwerveSubsystem swerveSubsystem) {


        return AutoBuilder.pathfindToPose(targetPose, constraints);

        }
    public static Command pathfindToPath(PathPlannerPath path) {
        return AutoBuilder.pathfindThenFollowPath(path, constraints);
    }

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
          return 
          timer.hasElapsed(trajectory.getTotalTime())
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
