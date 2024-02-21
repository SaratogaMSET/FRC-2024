// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterIOReal;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Turret.TurretIO;
import frc.robot.subsystems.Turret.TurretIOReal;
import frc.robot.subsystems.Turret.TurretIOSim;

public class RobotContainer {
  ShooterIO shooterIO = Robot.isReal() ? new ShooterIOReal() : new ShooterIOSim();
  TurretIO turretIO = Robot.isReal() ? new TurretIOReal() : new TurretIOSim();
  ShooterSubsystem shooter= new ShooterSubsystem(shooterIO, turretIO);
  CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {
    controller.a().onTrue(shooter.run(()->shooter.setTurretPDF(Math.toRadians(50),0)));
    // .onFalse(shooter.run(()->shooter.setTurretPDF(Math.toRadians(0),0)));

    controller.b().onTrue(shooter.run(()->shooter.setPivotPDF(Math.toRadians(30),0)));
    // .onFalse(shooter.run(()->shooter.setPivotPDF(Math.toRadians(0),0)));
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
