package frc.robot.commands.Autos;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Intake.DesiredStates;
import frc.robot.Constants.Intake.DesiredStates.ArmStates;
import frc.robot.commands.Intake.IntakeDefaultCommand;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;


public class AutoPathHelper {

    public static Command annotateName(Command x, String pathToFollow){
        x.setName(pathToFollow);
        return x;
    }

    public static Command followPath(String pathToFollow){
        return annotateName(AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory(pathToFollow)), pathToFollow);
    }
    public static Command followPathWhileIntaking(String pathToFollow, IntakeSubsystem intake, ArmStates armStates){
        return annotateName(
                followPath(pathToFollow).alongWith(new IntakeDefaultCommand(intake, armStates)),
                pathToFollow);
    } 

     public static Command followPathWhileShooting(String pathToFollow, ShooterSubsystem shooterSubsystem, SwerveSubsystem swerve){
        return annotateName(followPath(pathToFollow)
        .alongWith(new ShooterCommand(shooterSubsystem, ()->swerve.getPose() , ()-> swerve.getFieldRelativeSpeeds())), pathToFollow);
    }
     public static Command followPathAfterShooting(String pathToFollow, ShooterSubsystem shooterSubsystem, SwerveSubsystem swerve){
        return annotateName(followPath(pathToFollow)
        .beforeStarting(new ShooterCommand(shooterSubsystem, ()-> swerve.getPose(), ()-> swerve.getFieldRelativeSpeeds())), pathToFollow);
    }

    public static Command sequencePaths(SwerveSubsystem swerve, Command... paths) {
        return Commands.runOnce(()-> swerve.setPose(PathPlannerPath.fromChoreoTrajectory(paths[0].getName()).getPreviewStartingHolonomicPose()), swerve).andThen(paths);
    }
}
