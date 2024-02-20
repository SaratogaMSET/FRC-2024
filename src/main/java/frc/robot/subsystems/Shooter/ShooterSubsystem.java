// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import frc.robot.Constants.ShooterFlywheelConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Turret.TurretIOInputsAutoLogged;
import frc.robot.subsystems.Turret.TurretIOReal;
import frc.robot.Constants;
import frc.robot.Constants.ShooterAnglerConstants;
import frc.robot.Constants.ShooterFeederConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;

public class ShooterSubsystem extends SubsystemBase {
  public ShooterIOReal shooterIO = new ShooterIOReal();
  public ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  public TurretIOReal turretIO = new TurretIOReal();
  public TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();

  public ShooterSubsystem() {
  }


  public double thetaRad(){
    return shooterInputs.theta; 
  }
  public double thetaDegrees(){
    return shooterInputs.theta * 180 / Math.PI;
  }
  public double thetaRadPerSec(){
      return shooterInputs.thetaRadPerSec;
  }
  public boolean beamBreak(){
      return shooterInputs.beamBreakTriggered;
    }
  //TODO: motor RPS vs output RPS, if geared
  public double rpsShooterAvg(){
      return (shooterInputs.shooterRPS[0] + shooterInputs.shooterRPS[1])/2;
  }
  public double rpmShooterAvg(){
      return (shooterInputs.shooterRPS[0] + shooterInputs.shooterRPS[1]) * 30;
  }
  public double voltageShooterLeft(){
      return shooterInputs.shooterAppliedVolts[0];
  }
  public double voltageShooterRight(){
      return shooterInputs.shooterAppliedVolts[1];
  }
  public double voltageAngle(){
      return shooterInputs.anglerAppliedVolts;
  }
  public boolean isShooterRunning() {
      return Math.abs(shooterInputs.shooterRPS[0]) + Math.abs(shooterInputs.shooterRPS[1]) > 0.1;
  }
  public double voltage(){
    return turretInputs.voltage;
  } 
  //TODO: Calibrate Zero Positions
  public double phiRad(){
      return turretInputs.phi; 
  }
  public double phiDegrees(){
      return turretInputs.phi * 180 / Math.PI;
    }
  //TOOD: Add gear ratio
  public double phiRadPerSec(){
      return turretInputs.phiRadPerSec;
  }
  public boolean isTurretRunning() {
    // Query some boolean state, such as a digital sensor.
    return Math.abs(phiRadPerSec()) > 0.01;
  }
  public double[] maxAngleFromShooter(double shooterAngle){
    return new double[]{Constants.TurretConstants.kLowerBound, Constants.TurretConstants.kHigherBound}; //TODO: DEPENDENCY REGRESSION FROM SHOOTER ANGLE
  }
  public boolean[] speedCompensatedBoundsShooter(){
    double projection = thetaRad() + thetaRadPerSec() * 0.1;
    return new boolean[]{projection < ShooterAnglerConstants.kLowerBound, projection > ShooterAnglerConstants.kHigherBound};
  }
  public boolean[] speedCompensatedBoundsTurret(){
    double projection = phiRad() + phiRadPerSec() * 0.1;
    return new boolean[]{projection < TurretConstants.kLowerBound, projection > TurretConstants.kHigherBound}; //TODO: DEPENDENCY REGRESSION FROM SHOOTER ANGLE
  }

  public void setShooterVoltage(double voltage){
    shooterIO.setShooterVoltage(voltage);
  }
  public void setAnglerVoltage(double voltage){
    //TODO: Factor in velocity, if velocity will hit it in N control iterations, reduce by a factor based on how quickly it would hit based on current velocity
    //TODO: BOUNDS, FEEDFORWARD in NONPRIMITIVE
    boolean[] boundsTriggered = speedCompensatedBoundsShooter();
    if(boundsTriggered[0] && voltage < 0){
      voltage = 0;
    }

    if(boundsTriggered[1] && voltage > 0){
      voltage = 0;
    }

    shooterIO.setAnglerVoltage(voltage);
  }
  public void setFeederVoltage(double voltage){
    shooterIO.setFeederVoltage(voltage);
  }
  public void setTurretVoltage(double voltage){
    //TODO: Tune RPS constant
    boolean[] boundsTriggered = speedCompensatedBoundsTurret();
    if(boundsTriggered[0] && voltage < 0){
        voltage = 0;
    }
    if(boundsTriggered[1] && voltage > 0){
        voltage = 0;
    }
    turretIO.setVoltage(voltage);
  }
  
  public void spinShooter(double targetRPM, double acceleration){
    double feedforward = ShooterFlywheelConstants.kV * targetRPM + ShooterFlywheelConstants.kA * acceleration + Math.signum(targetRPM) * ShooterFlywheelConstants.kF;
    double feedback = (targetRPM - rpmShooterAvg()) * ShooterFlywheelConstants.kP + acceleration * ShooterFlywheelConstants.kD;
    double controlVoltage = feedforward + feedback;
    
    if(Math.abs(controlVoltage) > ShooterFlywheelConstants.kVoltageMax) controlVoltage = Math.signum(controlVoltage) * ShooterFlywheelConstants.kVoltageMax;
    setShooterVoltage(controlVoltage);
  }
  public void setPhiPDF(double targetRad, double target_radPerSec){
    targetRad = MathUtil.clamp(targetRad, ShooterAnglerConstants.kLowerBound, ShooterAnglerConstants.kHigherBound);
    double error = targetRad - thetaRad();
    double voltagePosition = ShooterAnglerConstants.kP * error + ShooterAnglerConstants.kD * thetaRadPerSec();
    double voltageVelocity = ShooterAnglerConstants.kV * target_radPerSec + ShooterAnglerConstants.kVP * (target_radPerSec - thetaRadPerSec());
    //Friction correction applies when outside tolerance
    double frictionTolerance = 1 * Math.PI / 180;
    if(Math.abs(error) > frictionTolerance) voltagePosition += ShooterAnglerConstants.kF * Math.signum(error);
    setAnglerVoltage(voltagePosition + voltageVelocity);
  }
  public void setThetaPDF(double target_rad, double target_radPerSec){
    target_rad = MathUtil.clamp(target_rad, Constants.TurretConstants.kLowerBound, Constants.TurretConstants.kHigherBound);
    double error = target_rad - phiRad();
    double voltagePosition = Constants.TurretConstants.kP * error + Constants.TurretConstants.kD * phiRadPerSec();
    double voltageVelocity = Constants.TurretConstants.kV * target_radPerSec + Constants.TurretConstants.kVP * (target_radPerSec - phiRadPerSec());
    //Friction correction applies when outside tolerance
    double frictionTolerance = 1 * Math.PI / 180; //TODO: TUNE
    if(Math.abs(error) > frictionTolerance) voltagePosition += Constants.TurretConstants.kF * Math.signum(error);
    setTurretVoltage(voltagePosition + voltageVelocity);
  }

  public Command shooterVoltage(double voltageLeft, double voltageRight) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setShooterVoltage(voltageLeft);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterIO.updateInputs(shooterInputs);    
    turretIO.updateInputs(turretInputs);
    Logger.processInputs("Shooter", shooterInputs);
    Logger.processInputs("Turret", turretInputs);

    reportNumber("Shooter RPM", rpmShooterAvg());
    reportNumber("Theta", thetaRad());
    reportNumber("Theta Speed", thetaRadPerSec() * 60/(2 * Math.PI));
    reportNumber("Voltage/Left", voltageShooterLeft());
    reportNumber("Voltage/Right", voltageShooterRight());

    reportNumber("Phi", phiDegrees());
    reportNumber("Phi Speed", phiRadPerSec() * 60);
    reportNumber("Voltage", voltage());

    SmartDashboard.putBoolean("Shooter/Bounds/ShooterLow", speedCompensatedBoundsShooter()[0]);
    SmartDashboard.putBoolean("Shooter/Bounds/ShooterHigh", speedCompensatedBoundsShooter()[1]);
    SmartDashboard.putBoolean("Shooter/Bounds/TurretLow", speedCompensatedBoundsTurret()[0]);
    SmartDashboard.putBoolean("Shooter/Bounds/TurretHigh", speedCompensatedBoundsTurret()[1]);
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
