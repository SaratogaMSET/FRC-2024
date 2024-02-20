// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;

public class ShooterSubsystem extends SubsystemBase {
  public ShooterIOReal io = new ShooterIOReal();
  public ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private double previousTime;
  private double previousRPS;

  private double acceleration;

  public ShooterSubsystem() {
  }


  //Radians
  public double angleRad(){
    return inputs.theta; 
  }
  public double angleRadPerSec(){
      return inputs.thetaRadPerSec;
  }
  public boolean beamBreak(){
      return inputs.beamBreakTriggered;
    }
  //TODO: motor RPS vs output RPS, if geared
  public double rpsAvg(){
      return (inputs.shooterRPS[0] + inputs.shooterRPS[1])/2;
  }
  public double rpmAvg(){
      return (inputs.shooterRPS[0] + inputs.shooterRPS[1]) * 30;
  }
  public double voltageLeft(){
      return inputs.shooterAppliedVolts[0];
  }
  public double voltageRight(){
      return inputs.shooterAppliedVolts[1];
  }
  public double voltageAngle(){
      return inputs.anglerAppliedVolts;
  }
  public boolean isRunning() {
      return Math.abs(inputs.shooterRPS[0]) + Math.abs(inputs.shooterRPS[1]) > 0.1;
  }
  public boolean[] speedCompensatedBounds(){
      double projection = angleRad() + angleRadPerSec() * 0.1;
      return new boolean[]{projection < ShooterConstants.kLowerBound, projection > ShooterConstants.kHigherBound};
  }
  public void setShooterVoltage(double voltage){
    io.setShooterVoltage(voltage);
  }
  public void setAnglerVoltage(double voltage){
    //TODO: Factor in velocity, if velocity will hit it in N control iterations, reduce by a factor based on how quickly it would hit based on current velocity
    //TODO: BOUNDS, FEEDFORWARD in NONPRIMITIVE
    boolean[] boundsTriggered = speedCompensatedBounds();
    if(boundsTriggered[0] && voltage < 0){
      voltage = 0;
    }

    if(boundsTriggered[1] && voltage > 0){
      voltage = 0;
    }

    io.setAnglerVoltage(voltage);
  }
  public void setFeederVoltage(double voltage){
    io.setFeederVoltage(voltage);
  }
  public void spin(double targetRPM, double acceleration){
    double feedforward = Constants.ShooterConstants.flywheelKv * targetRPM + Constants.ShooterConstants.flywheelKa * acceleration + Math.signum(targetRPM) * Constants.ShooterConstants.flywheelKf;
    double feedback = (targetRPM - rpmAvg()) * Constants.ShooterConstants.flywheelKp + acceleration * Constants.ShooterConstants.flywheelKd;
    double controlVoltage = feedforward + feedback;
    
    if(Math.abs(controlVoltage) > Constants.ShooterConstants.flywheelMax) controlVoltage = Math.signum(controlVoltage) * Constants.ShooterConstants.flywheelMax;
    setShooterVoltage(controlVoltage);
  }

  public void setAnglePDF(double targetRad, double target_radPerSec){
    targetRad = MathUtil.clamp(targetRad, Constants.ShooterConstants.kLowerBound, Constants.ShooterConstants.kHigherBound);
    double error = targetRad - angleRad();
    double voltagePosition = Constants.ShooterConstants.anglerKp * error + Constants.ShooterConstants.anglerKd * angleRadPerSec();
    double voltageVelocity = Constants.ShooterConstants.anglerKv * target_radPerSec + Constants.ShooterConstants.anglerKvp * (target_radPerSec - angleRadPerSec());
    //Friction correction applies when outside tolerance
    double frictionTolerance = 1 * Math.PI / 180;
    if(Math.abs(error) > frictionTolerance) voltagePosition += Constants.ShooterConstants.anglerKf * Math.signum(error);
    setAnglerVoltage(voltagePosition + voltageVelocity);
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
    io.updateInputs(inputs);    
    Logger.processInputs("Shooter", inputs);
    double curTime = Timer.getFPGATimestamp();

    acceleration = (rpsAvg() - previousRPS)/(curTime - previousTime);

    reportNumber("Shooter RPM", rpmAvg());
    reportNumber("Angle RadPerSec", angleRadPerSec() * 60/(2 * Math.PI));
    reportNumber("Acceleration", acceleration);
    reportNumber("Voltage/Left", voltageLeft());
    reportNumber("Voltage/Right", voltageRight());

    previousTime = Timer.getFPGATimestamp();
    previousRPS = rpsAvg();
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
