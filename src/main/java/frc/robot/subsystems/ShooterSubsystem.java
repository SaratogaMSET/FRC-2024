// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.*;
// import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.numbers.N1;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class ShooterSubsystem extends SubsystemBase {
  TalonFX leftMotor;
  TalonFX rightMotor;
  TalonFX angleMotor;
  TalonFX feederMotor;

  CANcoder encoder;
  DigitalInput beamBreak;

  // LinearSystem<N1, N1, N1> plant;
  // LinearQuadraticRegulator<N1, N1, N1> controller;

  private double voltageLeft;
  private double voltageRight;
  private double voltageAngle;

  private double previousTime;
  private double previousRPS;

  private double acceleration;

  public ShooterSubsystem() {
    leftMotor = new TalonFX(Constants.ShooterConstants.kLeftMotorPort);
    rightMotor = new TalonFX(Constants.ShooterConstants.kRightMotorPort);
    angleMotor = new TalonFX(Constants.ShooterConstants.kAngleMotorPort);
    feederMotor = new TalonFX(Constants.ShooterConstants.kFeederMotorPort);

    configMotors();

    encoder = new CANcoder(Constants.ShooterConstants.kEncoderPort);
    beamBreak = new DigitalInput(Constants.ShooterConstants.kBeamBreakPort);

    // plant = LinearSystemId.identifyVelocitySystem(Constants.ShooterConstants.flywheelKv, Constants.ShooterConstants.flywheelKa);
    // Vector<N1> Q = VecBuilder.fill(1.0/(2 * 2));
    // Vector<N1> R = VecBuilder.fill(1.0/(1 * 1));
    // controller = new LinearQuadraticRegulator<N1, N1, N1>(plant, Q, R, Constants.dt);
  }

  public void configMotors(){
    TalonFXConfiguration generalConfig = new TalonFXConfiguration();
    MotorOutputConfigs motorConfig = new MotorOutputConfigs();
    ClosedLoopRampsConfigs voltageRampConfig = new ClosedLoopRampsConfigs();
    CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();

    motorConfig.withPeakForwardDutyCycle(1);
    motorConfig.withPeakReverseDutyCycle(1);
    
    voltageRampConfig.withVoltageClosedLoopRampPeriod(1);

    currentLimitConfig.withStatorCurrentLimit(40); //TODO: Fix/TUNE
    currentLimitConfig.withStatorCurrentLimitEnable(true);

    generalConfig.withMotorOutput(motorConfig);
    generalConfig.withClosedLoopRamps(voltageRampConfig);
    generalConfig.withCurrentLimits(currentLimitConfig);

    leftMotor.getConfigurator().apply(generalConfig);
    rightMotor.getConfigurator().apply(generalConfig);

    leftMotor.setInverted(false);
    rightMotor.setInverted(true);
    leftMotor.setControl(new CoastOut());
    rightMotor.setControl(new CoastOut());

    feederMotor.setInverted(false);
    angleMotor.setInverted(false);
    feederMotor.setControl(new CoastOut());
    angleMotor.setControl(new StaticBrake());
  }
//TODO: motor RPS vs output RPS, if geared
  public boolean isRunning() {
    return Math.abs(rpsLeft()) + Math.abs(rpsRight()) > 0.1;
  }
  public double rpsLeft(){
    return leftMotor.getVelocity().getValueAsDouble();
  }
  public double rpsRight(){
    return rightMotor.getVelocity().getValueAsDouble();
  }
  public double rpsAvg(){
    return (rpsLeft() + rpsRight())/2;
  }
  public double rpsAngle(){
    return angleMotor.getVelocity().getValueAsDouble(); //TODO:Gearing
  }
  public double voltageLeft(){
    return this.voltageLeft;
  }
  public double voltageRight(){
    return this.voltageRight;
  }
  public double voltageAngle(){
    return this.voltageAngle;
  }
  //TODO: Calibrate Zero Positions
  public double angle(){
    return encoder.getAbsolutePosition().getValueAsDouble() - Constants.ShooterConstants.kEncoderOffset; 
  }
  public boolean beamBreak(){
    return beamBreak.get();
  }
  public void setShooterVoltage(double voltageLeft, double voltageRight){
    leftMotor.setVoltage(voltageLeft);
    rightMotor.setVoltage(voltageRight);

    this.voltageLeft = voltageLeft;
    this.voltageRight = voltageRight;
  }
  public void setAngleVoltage(double voltage){
    //TODO: Factor in velocity, if velocity will hit it in N control iterations, reduce by a factor based on how quickly it would hit based on current velocity
    //TODO: BOUNDS, FEEDFORWARD in NONPRIMITIVE
    if(angle() + rpsAngle() * 0.1 < Constants.ShooterConstants.kLowerBound && voltage < 0){
      voltage = 0;
    }

    if(angle() + rpsAngle() * 0.1 > Constants.ShooterConstants.kHigherBound && voltage > 0){
      voltage = 0;
    }

    angleMotor.setVoltage(voltage);
  }
  public void setFeederVoltage(double voltage){
    feederMotor.setVoltage(voltage);
  }
  public void spin(double velocity, double acceleration){
    double feedforward = Constants.ShooterConstants.flywheelKv * velocity + Constants.ShooterConstants.flywheelKa * acceleration + Math.signum(velocity) * Constants.ShooterConstants.flywheelKf;
    double feedback = (velocity - rpsAvg()) * Constants.ShooterConstants.flywheelKp + acceleration * Constants.ShooterConstants.flywheelKd;
    double controlVoltage = feedforward + feedback;
    
    if(Math.abs(controlVoltage) > Constants.ShooterConstants.flywheelMax) controlVoltage = Math.signum(controlVoltage) * Constants.ShooterConstants.flywheelMax;
    setShooterVoltage(controlVoltage, controlVoltage);
  }
  public void setAnglePDF(double target_rad, double target_radPerSec){
    double error = target_rad - angle();
    double voltagePosition = Constants.ShooterConstants.anglerKp * error + Constants.ShooterConstants.anglerKd * rpsAngle();
    double voltageVelocity = Constants.ShooterConstants.anglerKv * target_radPerSec + Constants.ShooterConstants.anglerKvp * (target_radPerSec - rpsAngle());
    //Friction correction applies when outside tolerance
    double frictionTolerance = 1 * Math.PI / 180;
    if(Math.abs(error) > frictionTolerance) voltagePosition += Constants.ShooterConstants.anglerKf * Math.signum(error);
    setAngleVoltage(voltagePosition + voltageVelocity);
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command voltageCommand(double voltageLeft, double voltageRight) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setShooterVoltage(voltageLeft, voltageRight);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double curTime = Timer.getFPGATimestamp();

    acceleration = (rpsAvg() - previousRPS)/(curTime - previousTime);

    reportNumber("RPM/Left", rpsLeft() * 60);
    reportNumber("RPM/Right", rpsRight() * 60);
    reportNumber("ACC", acceleration);
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
