package frc.robot.commands.Autos;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Intake.DesiredStates.Ground;
import frc.robot.commands.Intake.IntakeNeutralCommand;
import frc.robot.commands.Intake.IntakePositionCommand;
import frc.robot.commands.Intake.RollerCommand;
import frc.robot.commands.Shooter.AimTestCommand;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.Roller.RollerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.util.AllianceFlipUtil;
import static frc.robot.subsystems.Swerve.Module.WHEEL_RADIUS;


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
        .alongWith(new AimTestCommand(swerve, shooterSubsystem, ()->swerve.getPose() , ()-> swerve.getFieldRelativeSpeeds(), roller, true, 9.0, true, false)), pathToFollow);
    }
     public static Command followPathAfterShooting(String pathToFollow, ShooterSubsystem shooterSubsystem, SwerveSubsystem swerve, RollerSubsystem roller){
        return annotateName(followPath(pathToFollow)
        .beforeStarting(new AimTestCommand(swerve, shooterSubsystem, ()-> swerve.getPose(), ()-> swerve.getFieldRelativeSpeeds(), roller, true, 9.0, true, false)), pathToFollow);
    }
    public static Command doPathAndIntakeThenShoot(Command path, SwerveSubsystem swerve, ShooterSubsystem shooterSubsystem, IntakeSubsystem intake, RollerSubsystem roller) {
        Command intakeCommand = new IntakePositionCommand(intake, Ground.LOWER_MOTION_SHOULDER_ANGLE, Ground.LOWER_MOTION_WRIST_ANGLE).withTimeout(1)
        // .alongWith(
        //   new RollerCommand(roller, 6, false, ()->intake.shoulderGetRads()).alongWith(shooterSubsystem.anglingDegrees(0.0,44))
        //   .andThen(Commands.run(()->roller.setShooterFeederVoltage(1.5), roller).withTimeout(1).until(()->roller.getShooterBeamBreak())))
          .andThen(new IntakeNeutralCommand(intake));

        Command out = Commands.deadline(path, intakeCommand);
        return out.beforeStarting(
            Commands.parallel(
                new AimTestCommand(swerve, shooterSubsystem, ()-> swerve.getPose(), ()-> swerve.getFieldRelativeSpeeds(), roller, true, 9.0, true, false),
                new SequentialCommandGroup(
                    new WaitCommand(2),
                    Commands.run(()-> roller.setShooterFeederVoltage(12), roller)
                )).withTimeout(3));        
    }
    public static Command sequencePaths(SwerveSubsystem swerve, Command... paths) {
        return Commands.runOnce(()-> swerve.setPose(AllianceFlipUtil.apply(PathPlannerPath.fromChoreoTrajectory(paths[0].getName()).getPreviewStartingHolonomicPose())), swerve).andThen(paths);
    }
    public static Command choreoCommand(ChoreoTrajectory traj, SwerveSubsystem swerve) {
        var thetaController = new PIDController(1.0, 0.0, 0.0);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        Command swerveCommand = Choreo.choreoSwerveCommand(
        traj,
        swerve::getPose,
        new PIDController(7, 0.0, 0.0),
        new PIDController(7, 0.0, 0.0),
        thetaController,
        (ChassisSpeeds speeds) -> swerve.runVelocity(speeds),
        () -> DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
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
}
