package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterCalculation;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ShooterCommand extends Command{
    ShooterCalculation solver = new ShooterCalculation();
    ShooterSubsystem shooterSubsystem;
    TurretSubsystem turretSubsystem;
    boolean previouslyInZone = false;
    double[] shotParams;

    boolean finishCommand = false;
    public ShooterCommand(ShooterSubsystem shooterSubsystem, TurretSubsystem turretSubsystem){
        this.shooterSubsystem = shooterSubsystem;
        this.turretSubsystem = turretSubsystem;
    }
    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
  public void initialize() {}
  public void execute() {
    //TODO: Update Solver State Based On Vision & Drive
    if(!previouslyInZone){
        shotParams = solver.solveAll();
    }else{
        shotParams = solver.solveWarmStart(shotParams[0], shotParams[1], shotParams[2]);
    }
    
    if(solver.shotWindupZone()){
        shooterSubsystem.spin(0, 0); //TODO: set shot velocity and get a LUT or wtv
        shooterSubsystem.setAnglePDF(shotParams[1], shotParams[4]);
        turretSubsystem.setAnglePDF(shotParams[0], shotParams[3]); //TODO: Convert from field to robot

        previouslyInZone = true;
    }else{
        previouslyInZone = false;
    }
    
    if(solver.shotZone()){
        double[] simulatedShot = solver.simulateShot(shotParams[0], shotParams[1], shotParams[2]);
        double shotErrorX = Math.abs(0 - simulatedShot[0]);
        double shotErrorY = Math.abs(0 - simulatedShot[1]);
        double shotErrorZ = Math.abs(0 - simulatedShot[2]);

        boolean isMonotonic = Math.sin(shotParams[1]) * solver.vMag - 9.806 * shotParams[2] > 0;
        if(shotErrorX < 0 && shotErrorY < 0 && shotErrorZ < 0 && isMonotonic){ //TODO: Include shooter velocity tolerance
            shooterSubsystem.setFeederVoltage(0); //TODO: Define Feeding Voltage
        }
        if(!shooterSubsystem.beamBreak()){
            finishCommand = true;
        }
    }
  }
  public void end(boolean interrupted) {}
  public boolean isFinished() {
    return finishCommand;
  }
}
