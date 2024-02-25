package frc.robot.commands.Shooter;

import java.util.concurrent.TimeoutException;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterCalculation;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.util.NoteVisualizer;

public class ShooterCommand extends Command{
    ShooterCalculation solver = new ShooterCalculation();
    ShooterSubsystem shooterSubsystem;
    boolean previouslyInZone = false;
    double[] shotParams;
    Timer timer = new Timer();
    boolean finishCommand = false;

    Supplier<Pose2d> robotPose;
    Supplier<ChassisSpeeds> chassisSpeeds;
    public ShooterCommand(ShooterSubsystem shooterSubsystem, Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> robotSpeeds){
        this.shooterSubsystem = shooterSubsystem;
        this.robotPose = robotPose;
        this.chassisSpeeds = robotSpeeds;
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
    solver.setState(pose.getX(), pose.getY(), pose.getRotation().getRadians(), NoteVisualizer.getIndexerPose3d().getZ(), chassisSpeeds.vxMetersPerSecond,
    chassisSpeeds.vyMetersPerSecond, 17.0);
    if(!previouslyInZone){
        shotParams = solver.solveAll();
    }else{
        shotParams = solver.solveWarmStart(shotParams[0], shotParams[1], shotParams[2]);
    }
    System.out.println("SP: " + shotParams[0] + " " + shotParams[1] + " " + shotParams[2]);
    if(solver.shotWindupZone()){
        shooterSubsystem.spinShooter(0, 0); //TODO: set shot velocity and get a LUT or wtv
        shooterSubsystem.setPivotPDF(shotParams[1], shotParams[4]);
        shooterSubsystem.setTurretPDF(shotParams[0], shotParams[3]); //TODO: Convert from field to robot

        previouslyInZone = true;
    }else{
        previouslyInZone = false;
    }
    
    if(solver.shotZone()){
        double[] simulatedShot = solver.simulateShot(shotParams[0], shotParams[1], shotParams[2]);
        NoteVisualizer.shoot().schedule();
        double shotErrorX = Math.abs(0 - simulatedShot[0]);
        double shotErrorY = Math.abs(0 - simulatedShot[1]);
        double shotErrorZ = Math.abs(0 - simulatedShot[2]);

        boolean isMonotonic = Math.sin(shotParams[1]) * solver.vMag - 9.806 * shotParams[2] > 0;
        if(shotErrorX < 0 && shotErrorY < 0 && shotErrorZ < 0 && isMonotonic){ //TODO: Include shooter velocity tolerance
            shooterSubsystem.setFeederVoltage(0); //TODO: Define Feeding Voltage
        }
        if(timer.get()>1){ //TODO: REMOVE ME
            finishCommand = true;
        }
        if(!shooterSubsystem.beamBreak()){
            finishCommand = true;
        }
    }
    // System.out.println("SP: " + shotParams[0] + " " + shotParams[1] + " " + shotParams[2]);
    // SmartDashboard.putNumberArray("Shooter/ShotParams", shotParams);
  }
  public void end(boolean interrupted) {}
  public boolean isFinished() {
    timer.stop();
    return finishCommand;
  }
}
