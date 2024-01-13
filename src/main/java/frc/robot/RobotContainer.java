// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Shooter.Shooter;
import frc.robot.Shooter.ShooterCommand;

public class RobotContainer {
    Shooter shooter = new Shooter();
    XboxController controller = new XboxController(0);
  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {
    //+, - for one roller set
    //-, + for two roller set
    shooter.setDefaultCommand(new ShooterCommand(shooter, ()-> (controller.getLeftY()), ()-> -(controller.getLeftY())));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
