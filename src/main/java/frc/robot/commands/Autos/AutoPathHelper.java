package frc.robot.commands.Autos;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.DesiredStates;
import frc.robot.commands.Intake.IntakePositionCommand;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.commands.Shooter.ShooterFixedTurretCommand;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.Roller.RollerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.util.AllianceFlipUtil;


public class AutoPathHelper {

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
        .alongWith(new ShooterCommand(shooterSubsystem, ()->swerve.getPose() , ()-> swerve.getFieldRelativeSpeeds(), roller, true, 9.0)), pathToFollow);
    }
     public static Command followPathAfterShooting(String pathToFollow, ShooterSubsystem shooterSubsystem, SwerveSubsystem swerve, RollerSubsystem roller){
        return annotateName(followPath(pathToFollow)
        .beforeStarting(new ShooterCommand(shooterSubsystem, ()-> swerve.getPose(), ()-> swerve.getFieldRelativeSpeeds(), roller, true, 9.0)), pathToFollow);
    }
    public static Command doPathAndIntakeThenShoot(Command path, SwerveSubsystem swerve, ShooterSubsystem shooterSubsystem, IntakeSubsystem intake, double shoulderAngle, double wristAngle, RollerSubsystem roller) {
        Command out = Commands.deadline(path, new IntakePositionCommand(intake, shoulderAngle, wristAngle));
        return out.beforeStarting(
            Commands.parallel(
                new ShooterCommand(shooterSubsystem, ()-> swerve.getPose(), ()-> swerve.getFieldRelativeSpeeds(), roller, true, 9.0),
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    Commands.run(()-> roller.setShooterFeederVoltage(12), roller)
                )).withTimeout(2));        
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
        new PIDController(7.5, 0.0, 0.0),
        new PIDController(7.5, 0.0, 0.0),
        thetaController,
        (ChassisSpeeds speeds) -> swerve.runVelocity(speeds),
        () -> DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        swerve
    );
    return swerveCommand;
    }
}
