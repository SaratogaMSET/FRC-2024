// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ShootOnMoveCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ShooterSubsystem m_shooter;
  private final TurretSubsystem m_turret;
  private DoubleSupplier driveX;
  private DoubleSupplier driveY;
  private DoubleSupplier driveVx;
  private DoubleSupplier driveVy;
  private DoubleSupplier targetX;
  private DoubleSupplier targetY;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShootOnMoveCommand(ShooterSubsystem shooter, TurretSubsystem turret, DoubleSupplier driveX, DoubleSupplier driveY, DoubleSupplier driveVx, DoubleSupplier driveVy, DoubleSupplier targetX, DoubleSupplier targetY) {
    m_shooter = shooter;
    m_turret = turret;

    this.driveX = driveX;
    this.driveY = driveY;
    this.driveVx = driveVx;
    this.driveVy = driveVy;
    this.targetX = targetX;
    this.targetY = targetY;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetXCompensated = targetX.getAsDouble() - driveVx.getAsDouble() * Constants.ShooterConstants.Regression.flightTime;
    double targetYCompensated = targetY.getAsDouble() - driveVy.getAsDouble() * Constants.ShooterConstants.Regression.flightTime;

    double displacementX = targetXCompensated - driveX.getAsDouble();
    double displacementY = targetYCompensated - driveY.getAsDouble();
    double displacement = Math.hypot(displacementX, displacementY);

    double projectionVx = driveVx.getAsDouble() * displacementX / (displacement * displacement) * displacementX;
    double projectionVy = driveVy.getAsDouble() * displacementY / (displacement * displacement) * displacementY;

    double displacementCompensated = Math.hypot(
      displacementX - projectionVx * Constants.ShooterConstants.Regression.flightTime,
      displacementY - projectionVy * Constants.ShooterConstants.Regression.flightTime);

    double phi = m_shooter.angleFromDistance(displacementCompensated);
    double theta = Math.atan2(displacementY, displacementX); //minus angle from robot to make it robot relative
    double shooterVelocity = m_shooter.velocityFromDistance(displacementCompensated);
    boolean velocityWithinTolerance = m_shooter.spin(shooterVelocity, 0);
    boolean phiWithinTolerance = m_shooter.setAnglePDF(phi * 180 / Math.PI);
    boolean thetaWithinTolerance = m_turret.setAnglePDF(theta * 180 / Math.PI);

    if(velocityWithinTolerance && phiWithinTolerance && thetaWithinTolerance){
      //TODO: Feed command here
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.setShooterVoltage(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
