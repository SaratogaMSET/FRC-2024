package frc.robot.commands.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterFlywheelConstants;
import frc.robot.Robot;
import frc.robot.subsystems.Shooter.ShooterCalculation;
import frc.robot.subsystems.Shooter.ShooterParameters;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.util.NoteVisualizer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AimTestCommand extends Command {
  ShooterCalculation solver = new ShooterCalculation();
  ShooterSubsystem shooterSubsystem;
  SwerveSubsystem swerve;
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
  double additionalRPM;
  Supplier<Pose2d> robotPose;
  Supplier<ChassisSpeeds> chassisSpeeds;

  public AimTestCommand(
      ShooterSubsystem shooterSubsystem,
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> robotSpeeds,
      boolean compensateGyro,
      double vMag,
      boolean shootSpeaker,
      boolean teleop,
      boolean autoShootInTeleop,
      double additionalRPM) {
    this.shooterSubsystem = shooterSubsystem;
    this.robotPose = robotPose;
    this.chassisSpeeds = robotSpeeds;
    this.compensateGyro = compensateGyro;
    this.vMag = vMag; // MPS
    this.shootSpeaker = shootSpeaker;
    this.teleop = teleop;
    this.autoShootInTeleop = autoShootInTeleop;
    this.additionalRPM = additionalRPM;

    Pose2d pose = robotPose.get();
    ChassisSpeeds chassisSpeeds = robotSpeeds.get();

    solver.setTarget(teleop, !shootSpeaker);
    solver.setState(
        pose.getX(),
        pose.getY(),
        ShooterFlywheelConstants.height,
        pose.getRotation().getRadians(),
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        vMag);

    shotParams = solver.solveAll(teleop, !shootSpeaker);

    addRequirements(shooterSubsystem);
  }

  /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  public void initialize() {
    timer.reset();
    timer.start();
  }

  public void execute() {
    Pose2d pose = robotPose.get();
    ChassisSpeeds chassisSpeeds = this.chassisSpeeds.get();
    Logger.recordOutput("Passed in Shooter Pose", pose);
    solver.setState(
        pose.getX(),
        pose.getY(),
        ShooterFlywheelConstants.height,
        pose.getRotation().getRadians(),
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        vMag);

    /* HELLO TO WHOEVER IS READING THIS. :3 THE SHOOTER DOESN'T WORK WITHOUT THE CONSTNAT SOLVE. DO I KNOW WHY? ABSOLUTELY NOT.
     * MAYBE SOMETHING IS WRONG WITH THE SOLVER BUT IT JUST DOESN'T RETURN EVEN REMOTELY RIGHT VALUES(THE COMMAND ENTIRELY DOESN'T RUN SOMETIMES,
     * TURNS THE WRONG DIRECTION... AND MORE. ESPECIALLY IN AUTO) - J.Z
     */
    if (!previouslyInZone) { // If not already aiming / solve cold start. Otherwise go off the warm
      // start(smaller correction needed)
      shotParams = solver.solveAll(teleop, !shootSpeaker);
    } else {
      shotParams =
          solver.solveWarmStart(shotParams[0], shotParams[1], shotParams[2], teleop, !shootSpeaker);
    }

    /* END OF PERIODIC SOLVE(?) CODE */
    if (solver.shotWindupZone()) {
      // if (teleop) {
      //   // swerve.setDriveCurrentLimit(30); //do we still want this
      // }
      double phi;
      if (compensateGyro) {
        phi =
            -(MathUtil.angleModulus(shotParams[0] + Math.PI - pose.getRotation().getRadians()))
                + Math.toRadians(1.5);
      } else {
        phi = -(MathUtil.angleModulus(0)) + Math.toRadians(4);
      }
      shooterSubsystem.spinShooterMPS(vMag, additionalRPM);
      shooterSubsystem.setPivotProfiled(shotParams[1], shotParams[4]); // Theta
      shooterSubsystem.setTurretProfiled(
          phi, shotParams[3] - chassisSpeeds.omegaRadiansPerSecond); // Phi

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
        simulatedShot =
            solver.simulateShotWithOverrideV(
                Math.PI - shooterSubsystem.turretRad() + pose.getRotation().getRadians(),
                shooterSubsystem.pivotRad(),
                shotParams[2],
                ShooterParameters.voltage_to_mps(
                    ShooterParameters.kRPM_to_voltage(shooterSubsystem.rpmShooterAvg() / 1000)));
      }

      NoteVisualizer.shoot(solver, simulatedShot).schedule();
      double shotErrorX = Math.abs(solver.targetX - simulatedShot[0]);
      double shotErrorY = Math.abs(solver.targetY - simulatedShot[1]);
      double shotErrorZ = Math.abs(solver.targetZ - simulatedShot[2]);

      Logger.recordOutput("AutoShot/targetPhi", shotParams[0] * 180 / Math.PI);
      Logger.recordOutput("AutoShot/targetTheta", shotParams[1] * 180 / Math.PI);
      Logger.recordOutput("AutoShot/targetT", shotParams[2]);

      Logger.recordOutput("AutoShot/shotErrorX", shotErrorX);
      // Logger.recordOutput("AutoShot/simShotX", simulatedShot[0]);
      Logger.recordOutput("AutoShot/targetShotX", solver.targetX);
      Logger.recordOutput("AutoShot/shotErrorY", shotErrorY);
      Logger.recordOutput("AutoShot/shotErrorZ", shotErrorZ);
      boolean isMonotonic = Math.sin(shotParams[1]) * solver.vMag - 9.806 * shotParams[2] > 0;
      double shooterErrorRPM =
          Math.abs(
              shooterSubsystem.rpmShooterAvg()
                  - ShooterParameters.mps_to_kRPM(vMag) * 1000
                  - additionalRPM);
      Logger.recordOutput("AutoShot/shotErrorRPM", shooterErrorRPM);
      Logger.recordOutput("AutoShot/Shooter Target", solver.retrieveTarget());

      Logger.recordOutput(
          "AutoShot/Transl Criteria", shotErrorX < 0.1 && shotErrorY < 0.1 && shotErrorZ < 0.08);
      Logger.recordOutput("AutoShot/Monotonic Criteria", isMonotonic);
      // Logger.recordOutput("AutoShot/BeamBreak Criteria", roller.getShooterBeamBreak());
      Logger.recordOutput("AutoShot/Input Criteria", (!teleop || autoShootInTeleop));
      // if ((shotErrorX < 0.1 && shotErrorY < 0.1 && shotErrorZ < 0.08 && isMonotonic
      //     ) && (!teleop || autoShootInTeleop)) {
      //   roller.setShooterFeederVoltage(12);
      //   startShot = true;
      // }
      // if(startShot){
      //   roller.setShooterFeederVoltage(12);
      // }
      // if(startShot && !roller.getShooterBeamBreak()){
      //   finishCommand = true;
      // }

    }
  }

  public void end(boolean interrupted) {
    startShot = false;
    finishCommand = false;
    if (DriverStation.isAutonomousEnabled()) {
      shooterSubsystem.spinShooterMPS(0, 0);
    } else {
      shooterSubsystem.setShooterVoltage(0);
    }
    // swerve.setDriveCurrentLimit(80.0);
  }

  public boolean isFinished() {
    timer.stop();
    return finishCommand;
  }
}
