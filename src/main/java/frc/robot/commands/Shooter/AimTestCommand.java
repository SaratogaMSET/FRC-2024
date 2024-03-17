package frc.robot.commands.Shooter;

import java.util.concurrent.TimeoutException;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Intake.Roller;
import frc.robot.Robot;
import frc.robot.Constants.ShooterFlywheelConstants;
import frc.robot.subsystems.Intake.Roller.RollerSubsystem;
import frc.robot.subsystems.Shooter.ShooterCalculation;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.NoteVisualizer;

public class AimTestCommand extends Command{
    ShooterCalculation solver = new ShooterCalculation();
    ShooterSubsystem shooterSubsystem;
    RollerSubsystem roller;
    boolean previouslyInZone = false;
    double[] shotParams;
    double vMag;
    boolean shootSpeaker;
    Timer timer = new Timer();
    boolean finishCommand = false;
    boolean compensateGyro;
    Supplier<Pose2d> robotPose;
    Supplier<ChassisSpeeds> chassisSpeeds;
    public AimTestCommand(ShooterSubsystem shooterSubsystem, Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotSpeeds, RollerSubsystem roller, boolean compensateGyro, double vMag, boolean shootSpeaker){
        this.shooterSubsystem = shooterSubsystem;
        this.roller = roller;
        this.robotPose = robotPose;
        this.chassisSpeeds = robotSpeeds;
        this.compensateGyro = compensateGyro;
        this.vMag = vMag;
        this.shootSpeaker = shootSpeaker;

        Pose2d pose = robotPose.get();
        ChassisSpeeds chassisSpeeds = robotSpeeds.get();
        solver.setStateSpeaker(pose.getX(), pose.getY(), ShooterFlywheelConstants.height, pose.getRotation().getRadians(),
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond, vMag);
        shotParams = solver.solveAll(shootSpeaker);

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
    SmartDashboard.putNumberArray("ShooterCommand passed in Pose", new double[]{pose.getX(), pose.getY(), pose.getRotation().getRadians()});
    solver.setStateSpeaker(pose.getX(), pose.getY(), ShooterFlywheelConstants.height, pose.getRotation().getRadians(),
    chassisSpeeds.vxMetersPerSecond,
    chassisSpeeds.vyMetersPerSecond, vMag);
    if(!previouslyInZone){
      System.out.println("Cold Start");
        shotParams = solver.solveAll(shootSpeaker);
    }else{
        System.out.println("Warm Start");
        shotParams = solver.solveWarmStart(shotParams[0], shotParams[1], shotParams[2], shootSpeaker);
    }
    System.out.println("SP: " + shotParams[0] + " " + shotParams[1] + " " + shotParams[2]);
    if(solver.shotWindupZone()){
      Logger.recordOutput("CurrentRotRadians",  pose.getRotation().getRadians());
      shooterSubsystem.spinShooterMPS(vMag);
      shooterSubsystem.setPivotPDF(shotParams[1], shotParams[4]);
      double phi; 
      if(compensateGyro){
        if(AllianceFlipUtil.shouldFlip()) phi = MathUtil.angleModulus(shotParams[0] + Math.PI + pose.getRotation().getRadians()) + Math.toRadians(4);
        else phi = -(MathUtil.angleModulus(shotParams[0] + Math.PI - pose.getRotation().getRadians())) + Math.toRadians(4);
      }
      else{
        if(AllianceFlipUtil.shouldFlip()) phi = (MathUtil.angleModulus(shotParams[0] + Math.PI)) + Math.toRadians(4);
        else phi = -(MathUtil.angleModulus(shotParams[0] + Math.PI)) + Math.toRadians(4);
      }

      Logger.recordOutput("AIMTEST PHI Desired", phi);
      Logger.recordOutput("AIMTEST PHI", MathUtil.angleModulus(shooterSubsystem.turretRad() - pose.getRotation().getRadians()));

      shooterSubsystem.setTurretPDF(phi, shotParams[3]);

      previouslyInZone = true;
    }else{
        previouslyInZone = false;
    }
    
    if(solver.shotZone()){
      double[] simulatedShot;
      //SIM:
      if(Robot.isSimulation()){
        simulatedShot = solver.simulateShot(shotParams[0], shotParams[1], shotParams[2]);
      }else{
        if(AllianceFlipUtil.shouldFlip()) simulatedShot = solver.simulateShot(Math.PI + shooterSubsystem.turretRad() + pose.getRotation().getRadians() + Math.toRadians(4), shooterSubsystem.pivotRad(), shotParams[2]);
        else simulatedShot = solver.simulateShot( Math.PI - shooterSubsystem.turretRad() + pose.getRotation().getRadians() + Math.toRadians(4), shooterSubsystem.pivotRad(), shotParams[2]);
        
      }
      Logger.recordOutput("AIMTEST sim", shotParams[0]);
      Logger.recordOutput("AIMTEST real", Math.PI - shooterSubsystem.turretRad() + pose.getRotation().getRadians() + Math.toRadians(4));
        NoteVisualizer.shoot(solver, simulatedShot).schedule();
        double shotErrorX = Math.abs(solver.targetX - simulatedShot[0]);
        double shotErrorY = Math.abs(solver.targetY - simulatedShot[1]);
        double shotErrorZ = Math.abs(solver.targetZ - simulatedShot[2]);

        Logger.recordOutput("shotErrorX", shotErrorX);
        Logger.recordOutput("simShotX", simulatedShot[0]);
        Logger.recordOutput("targetShotX", solver.targetX);
        Logger.recordOutput("shotErrorY", shotErrorY);
        Logger.recordOutput("shotErrorZ", shotErrorZ);
        boolean isMonotonic = Math.sin(shotParams[1]) * solver.vMag - 9.806 * shotParams[2] > 0;
        if(shotErrorX < 0.05 && shotErrorY < 0.05 && shotErrorZ < 0.02 && isMonotonic){ //TODO: Include shooter velocity tolerance//TODO: Define Feeding Voltage
          // roller.setShooterFeederVoltage(12);
        }
    
        // if(!roller.getShooterBeamBreak()){
        //     finishCommand = true;
        // }
    }
    // System.out.println("SP: " + shotParams[0] + " " + shotParams[1] + " " + shotParams[2]);
    // SmartDashboard.putNumberArray("Shooter/ShotParams", shotParams);
  }
  public void end(boolean interrupted) {}
  public boolean isFinished() {
    timer.stop();
    return finishCommand;
    // return true;
  }
}
