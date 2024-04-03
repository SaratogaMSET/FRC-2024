package frc.robot.commands.Shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake;
import frc.robot.Constants.ShooterFlywheelConstants;
import frc.robot.Robot;
import frc.robot.subsystems.Intake.Roller.RollerSubsystem;
import frc.robot.subsystems.Shooter.ShooterCalculation;
import frc.robot.subsystems.Shooter.ShooterParameters;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.NoteVisualizer;

public class AimTestCommand extends Command {
  ShooterCalculation solver = new ShooterCalculation();
  ShooterSubsystem shooterSubsystem;
  SwerveSubsystem swerve;
  RollerSubsystem roller;
  boolean previouslyInZone = false;
  double[] shotParams;
  double vMag;
  boolean shootSpeaker;
  Timer timer = new Timer();
  boolean startShot = false;
  boolean finishCommand = false;
  boolean compensateGyro;
  boolean teleop;
  boolean autoShootInTeleop;
  Supplier<Pose2d> robotPose;
  Supplier<ChassisSpeeds> chassisSpeeds;

  public AimTestCommand(ShooterSubsystem shooterSubsystem, Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> robotSpeeds, RollerSubsystem roller, boolean compensateGyro, double vMag,
      boolean shootSpeaker, boolean teleop, boolean autoShootInTeleop) {
    this.shooterSubsystem = shooterSubsystem;
    this.roller = roller;
    this.robotPose = robotPose;
    this.chassisSpeeds = robotSpeeds;
    this.compensateGyro = compensateGyro;
    this.vMag = vMag;
    this.shootSpeaker = shootSpeaker;
    this.teleop = teleop;
    this.autoShootInTeleop = autoShootInTeleop;

    Pose2d pose = robotPose.get();
    ChassisSpeeds chassisSpeeds = robotSpeeds.get();
      
    solver.setTarget(teleop, !shootSpeaker);
    solver.setState(pose.getX(), pose.getY(), ShooterFlywheelConstants.height, pose.getRotation().getRadians(),
      chassisSpeeds.vxMetersPerSecond,
      chassisSpeeds.vyMetersPerSecond, vMag);

    shotParams = solver.solveAll(teleop, !shootSpeaker);

    addRequirements(shooterSubsystem);
  }

  /**
   * The initial subroutine of a command. Called once when the command is
   * initially scheduled.
   */
  public void initialize() {
    timer.reset();
    timer.start();
  }

  public void execute() {
    Pose2d pose = robotPose.get();
    ChassisSpeeds chassisSpeeds = this.chassisSpeeds.get();
    // SmartDashboard.putNumberArray("ShooterCommand passed in Pose",
    //     new double[] { pose.getX(), pose.getY(), pose.getRotation().getRadians() });
    Logger.recordOutput("Passed in Shooter Pose", pose);
    solver.setState(pose.getX(), pose.getY(), ShooterFlywheelConstants.height, pose.getRotation().getRadians(),
      chassisSpeeds.vxMetersPerSecond,
      chassisSpeeds.vyMetersPerSecond, vMag);
    
    /* HELLO TO WHOEVER IS READING THIS. :3 THE SHOOTER DOESN'T WORK WITHOUT THE CONSTNAT SOLVE. DO I KNOW WHY? ABSOLUTELY NOT.
     * MAYBE SOMETHING IS WRONG WITH THE SOLVER BUT IT JUST DOESN'T RETURN EVEN REMOTELY RIGHT VALUES(THE COMMAND ENTIRELY DOESN'T RUN SOMETIMES
     * TURNS THE WRONG DIRECTION... AND MORE. ESPECIALLY IN AUTO) - J.Z
     */
    if (!previouslyInZone) {
      // System.out.println("Cold Start");
      shotParams = solver.solveAll(teleop, !shootSpeaker);
    } else {
      // System.out.println("Warm Start");
      shotParams = solver.solveWarmStart(shotParams[0], shotParams[1], shotParams[2], teleop, !shootSpeaker);
    }

    /* END OF PERIODIC SOLVE(?) CODE */

    System.out.println("SP: " + shotParams[0] + " " + shotParams[1] + " " + shotParams[2]);
    if (solver.shotWindupZone()) {
      // Logger.recordOutput("CurrentRotRadians", pose.getRotation().getRadians());
      // if (teleop) {
      //   // swerve.setDriveCurrentLimit(30); //do we still want this
      // }
      shooterSubsystem.spinShooterMPS(vMag); //TODO: ADD BACK
      shooterSubsystem.setPivotProfiled(shotParams[1], shotParams[4]); //shotparams[1]
      double phi;
      if (compensateGyro) {
        if (AllianceFlipUtil.shouldFlip() && !DriverStation.isAutonomous())
          phi = -MathUtil.angleModulus(shotParams[0] - pose.getRotation().getRadians()) + Math.toRadians(4);
        else
          phi = -(MathUtil.angleModulus(shotParams[0] + Math.PI - pose.getRotation().getRadians())) + Math.toRadians(4);
      } else {
        if (AllianceFlipUtil.shouldFlip() && !DriverStation.isAutonomous())
          phi =  -(MathUtil.angleModulus(0)) + Math.toRadians(4);
        else
          phi = -(MathUtil.angleModulus(0)) + Math.toRadians(4);
      }

      // Logger.recordOutput("AIMTEST PHI Desired", phi);
      // Logger.recordOutput("AIMTEST PHI",
          // MathUtil.angleModulus(shooterSubsystem.turretRad() - pose.getRotation().getRadians());

      shooterSubsystem.setTurretProfiled(phi, shotParams[3]); //phi

      previouslyInZone = true;
    } else {
      previouslyInZone = false;
    }

    if (solver.shotZone()) {
      double[] simulatedShot;
      // SIM:
      if (Robot.isSimulation()) {
        simulatedShot = solver.simulateShot(shotParams[0], shotParams[1], shotParams[2]);
      } else {
        if (AllianceFlipUtil.shouldFlip() && false)
          simulatedShot = solver.simulateShot(
              Math.PI + shooterSubsystem.turretRad() + pose.getRotation().getRadians() + Math.toRadians(4),
              shooterSubsystem.pivotRad(), shotParams[2]);
        else
          simulatedShot = solver.simulateShot(
              Math.PI - shooterSubsystem.turretRad() + pose.getRotation().getRadians() + Math.toRadians(4),
              shooterSubsystem.pivotRad(), shotParams[2]);

      }
      // Logger.recordOutput("AIMTEST sim", shotParams[0]);
      // Logger.recordOutput("AIMTEST real",
          // Math.PI - shooterSubsystem.turretRad() + pose.getRotation().getRadians() + Math.toRadians(4));
      NoteVisualizer.shoot(solver, simulatedShot).schedule();
      double shotErrorX = Math.abs(solver.targetX - simulatedShot[0]);
      double shotErrorY = Math.abs(solver.targetY - simulatedShot[1]);
      double shotErrorZ = Math.abs(solver.targetZ - simulatedShot[2]);

      Logger.recordOutput("shotErrorX", shotErrorX);
      // Logger.recordOutput("simShotX", simulatedShot[0]);
      Logger.recordOutput("targetShotX", solver.targetX);
      Logger.recordOutput("shotErrorY", shotErrorY);
      Logger.recordOutput("shotErrorZ", shotErrorZ);
      boolean isMonotonic = Math.sin(shotParams[1]) * solver.vMag - 9.806 * shotParams[2] > 0;
      double shooterErrorRPM = Math.abs(shooterSubsystem.rpmShooterAvg() - ShooterParameters.mps_to_kRPM(vMag) * 1000);
      Logger.recordOutput("shotErrorRPM", shooterErrorRPM);
      Logger.recordOutput("Shooter Target", solver.retrieveTarget());

      if ((shotErrorX < 0.2 && shotErrorY < 0.2 && shotErrorZ < 0.0254 && isMonotonic && shooterErrorRPM < 40
          && roller.getShooterBeamBreak()) && (!teleop || autoShootInTeleop)) {
        roller.setShooterFeederVoltage(12);
        startShot = true;
      }
      if(startShot){
        roller.setShooterFeederVoltage(12);
      }
      if(startShot && !roller.getShooterBeamBreak()){
        finishCommand = true;
      }
      // if(!roller.getShooterBeamBreak()){
      // finishCommand = true;
      // }
    }
    // System.out.println("SP: " + shotParams[0] + " " + shotParams[1] + " " +
    // shotParams[2]);
    // SmartDashboard.putNumberArray("Shooter/ShotParams", shotParams);
  }

  public void end(boolean interrupted) {
    if(DriverStation.isAutonomousEnabled()){
      shooterSubsystem.spinShooterMPS(7.6);
    }
    else{
      shooterSubsystem.setShooterVoltage(0);
    }
    roller.setShooterFeederVoltage(0);
    // swerve.setDriveCurrentLimit(80.0);
  }

  public boolean isFinished() {
    timer.stop();
    return finishCommand;
    // return true;
  }
}
