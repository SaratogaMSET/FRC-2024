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
import frc.robot.subsystems.Intake.Roller.RollerSubsystem;
import frc.robot.subsystems.Shooter.ShooterCalculation;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.NoteVisualizer;

public class ShooterCommand extends Command{
    ShooterCalculation solver = new ShooterCalculation();
    ShooterSubsystem shooterSubsystem;
    RollerSubsystem roller;
    boolean previouslyInZone = false;
    double[] shotParams;
    double vMag = 9.0;
    Timer timer = new Timer();
    boolean finishCommand = false;
    boolean compensateGyro;
    Supplier<Pose2d> robotPose;
    Supplier<ChassisSpeeds> chassisSpeeds;
    public ShooterCommand(ShooterSubsystem shooterSubsystem, Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotSpeeds, RollerSubsystem roller, boolean compensateGyro, double vMag){
        this.shooterSubsystem = shooterSubsystem;
        this.roller = roller;
        this.robotPose = robotPose;
        this.chassisSpeeds = robotSpeeds;
        this.compensateGyro = compensateGyro;
        this.vMag = vMag;

        Pose2d pose = robotPose.get();
        ChassisSpeeds chassisSpeeds = robotSpeeds.get();
        solver.setState(pose.getX(), pose.getY(), 24 * 0.0254, pose.getRotation().getRadians(),
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond, vMag);
        shotParams = solver.solveAll();

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
    solver.setState(pose.getX(), pose.getY(), 24 * 0.0254, pose.getRotation().getRadians(),
    chassisSpeeds.vxMetersPerSecond,
    chassisSpeeds.vyMetersPerSecond, vMag);
    // if(!previouslyInZone){
    //   System.out.println("Cold Start");
    //     shotParams = solver.solveAll();
    // }else{
    //     System.out.println("Warm Start");
    //     shotParams = solver.solveWarmStart(shotParams[0], shotParams[1], shotParams[2]);
    // }
    System.out.println("SP: " + shotParams[0] + " " + shotParams[1] + " " + shotParams[2]);
    if(solver.shotWindupZone()){
      Logger.recordOutput("CurrentRotRadians",  pose.getRotation().getRadians());
        shooterSubsystem.spinShooterMPS(vMag);
        shooterSubsystem.setPivotPDF(shotParams[1], shotParams[4]);
        double phi; 
        if(compensateGyro){
          if(AllianceFlipUtil.shouldFlip()) phi = (MathUtil.angleModulus(shotParams[0] + Math.PI) + pose.getRotation().getRadians()) + Math.toRadians(4);
          else phi = -(MathUtil.angleModulus(shotParams[0] + Math.PI) - pose.getRotation().getRadians()) + Math.toRadians(4);
        }
        else{
          if(AllianceFlipUtil.shouldFlip()) phi = (MathUtil.angleModulus(shotParams[0] + Math.PI)) + Math.toRadians(4);
          else phi = -(MathUtil.angleModulus(shotParams[0] + Math.PI)) + Math.toRadians(4);
        }
        Logger.recordOutput("desired phi Shooter Command", phi);
        shooterSubsystem.setTurretPDF(phi, shotParams[3]); //  - pose.getRotation().getRadians() for on the robot

        previouslyInZone = true;
    }else{
        previouslyInZone = false;
    }
    
    if(solver.shotZone()){
      //REAL: double[] simulatedShot = solver.simulateShot(shooterSubsystem.turretRad(), shooterSubsystem.pivotRad(), shotParams[2]);
      //SIM:
        double[] simulatedShot = solver.simulateShot(Math.PI + shotParams[0], shotParams[1], shotParams[2]);
        NoteVisualizer.shoot(solver, simulatedShot).schedule();
        double shotErrorX = Math.abs(0 - simulatedShot[0]);
        double shotErrorY = Math.abs(0 - simulatedShot[1]);
        double shotErrorZ = Math.abs(0 - simulatedShot[2]);

        Logger.recordOutput("shotErrorX", shotErrorX);
        // Logger.recordOutput("shotErrorY", shotErrorY);
        // Logger.recordOutput("shotErrorZ", shotErrorZ);
        boolean isMonotonic = Math.sin(shotParams[1]) * solver.vMag - 9.806 * shotParams[2] > 0;
        if(shotErrorX < 0 && shotErrorY < 0 && shotErrorZ < 0 && isMonotonic){ //TODO: Include shooter velocity tolerance//TODO: Define Feeding Voltage
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
