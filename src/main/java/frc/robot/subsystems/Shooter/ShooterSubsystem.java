// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.MathUtil;

public class ShooterSubsystem extends SubsystemBase {
  public ShooterIOReal io = new ShooterIOReal();
  private double previousTime;
  private double previousRPS;

  private double acceleration;

  public ShooterSubsystem() {
  }
  
  public void spin(double velocity, double acceleration){
    double feedforward = Constants.ShooterConstants.flywheelKv * velocity + Constants.ShooterConstants.flywheelKa * acceleration + Math.signum(velocity) * Constants.ShooterConstants.flywheelKf;
    double feedback = (velocity - io.rpsAvg()) * Constants.ShooterConstants.flywheelKp + acceleration * Constants.ShooterConstants.flywheelKd;
    double controlVoltage = feedforward + feedback;
    
    if(Math.abs(controlVoltage) > Constants.ShooterConstants.flywheelMax) controlVoltage = Math.signum(controlVoltage) * Constants.ShooterConstants.flywheelMax;
    io.setShooterVoltage(controlVoltage);
  }

  public void setAnglePDF(double target_rad, double target_radPerSec){
    target_rad = MathUtil.clamp(target_rad, Constants.ShooterConstants.kLowerBound, Constants.ShooterConstants.kHigherBound);
    double error = target_rad - io.angleRad();
    double voltagePosition = Constants.ShooterConstants.anglerKp * error + Constants.ShooterConstants.anglerKd * io.rpsAngle();
    double voltageVelocity = Constants.ShooterConstants.anglerKv * target_radPerSec + Constants.ShooterConstants.anglerKvp * (target_radPerSec - io.rpsAngle());
    //Friction correction applies when outside tolerance
    double frictionTolerance = 1 * Math.PI / 180;
    if(Math.abs(error) > frictionTolerance) voltagePosition += Constants.ShooterConstants.anglerKf * Math.signum(error);
    io.setAnglerVoltage(voltagePosition + voltageVelocity);
  }
  public void setFeederVoltage(double voltage){
    io.setFeederVoltage(voltage);
  }

  public Command shooterVoltage(double voltageLeft, double voltageRight) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          io.setShooterVoltage(voltageLeft);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double curTime = Timer.getFPGATimestamp();

    acceleration = (io.rpsAvg() - previousRPS)/(curTime - previousTime);

    reportNumber("RPM/Left", io.rpsLeft() * 60);
    reportNumber("RPM/Right",io.rpsRight() * 60);
    reportNumber("RPM/Angle", io.angleRadPerSec() * 60/(2 * Math.PI));
    reportNumber("ACC", acceleration);
    reportNumber("Voltage/Left", io.voltageLeft());
    reportNumber("Voltage/Right", io.voltageRight());

    previousTime = Timer.getFPGATimestamp();
    previousRPS = io.rpsAvg();
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  @Override
  public String getName(){
    return "Shooter Subsystem";
  }

  public void reportNumber(String name, double number){
    String prefix = "Shooter/";
    SmartDashboard.putNumber(prefix + name, number);
  }
}
