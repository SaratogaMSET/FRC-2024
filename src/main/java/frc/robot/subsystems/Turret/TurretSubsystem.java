// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurretSubsystem extends SubsystemBase {
  public TurretIOReal io = new TurretIOReal();
  public TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  public TurretSubsystem() {
  }

  public void setAnglePDF(double target_rad, double target_radPerSec){
    target_rad = MathUtil.clamp(target_rad, Constants.TurretConstants.kLowerBound, Constants.TurretConstants.kHigherBound);
    double error = target_rad - getAngleRadians();
    double voltagePosition = Constants.TurretConstants.kP * error + Constants.TurretConstants.kD * io.rps();
    double voltageVelocity = Constants.TurretConstants.kV * target_radPerSec + Constants.TurretConstants.kVP * (target_radPerSec - io.rps());
    //Friction correction applies when outside tolerance
    double frictionTolerance = 1 * Math.PI / 180;
    if(Math.abs(error) > frictionTolerance) voltagePosition += Constants.TurretConstants.kF * Math.signum(error);
    io.setVoltage(voltagePosition + voltageVelocity);
  }

  public boolean isRunning() {
    // Query some boolean state, such as a digital sensor.
    return Math.abs(io.rps()) > 0.1;
  }
  public double getAngleRadians(){
    return inputs.phi;
  }
  //TODO: FIX
  public boolean[] speedCompensatedBounds(){
    double projection = getAngleRadians() + io.rps() * 0.1;
    return new boolean[]{projection < Constants.TurretConstants.kLowerBound, projection > Constants.TurretConstants.kHigherBound};
}
  @Override
  public void periodic() {
    io.updateInputs(inputs);    
    Logger.processInputs("Turret", inputs);
    // This method will be called once per scheduler run
    reportNumber("Position", Math.toDegrees(getAngleRadians()));
    reportNumber("RPM", io.rps() * 60);
    reportNumber("Voltage", io.voltage());
    SmartDashboard.putBoolean("Turret/Bounds/Low", speedCompensatedBounds()[0]);
    SmartDashboard.putBoolean("Turret/Bounds/High", speedCompensatedBounds()[1]);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public String getName(){
    return "Turret Subsystem";
  }

  public void reportNumber(String name, double number){
    String prefix = "Turret/";
    SmartDashboard.putNumber(prefix + name, number);
  }
}
