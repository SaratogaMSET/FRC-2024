// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

<<<<<<< Updated upstream
package frc.robot.subsystems.Shooter;
=======
package frc.robot.subsystems;
>>>>>>> Stashed changes

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

<<<<<<< Updated upstream
public class ShooterSubsystem extends SubsystemBase {

  // LinearSystem<N1, N1, N1> plant;
  // LinearQuadraticRegulator<N1, N1, N1> controller;

  private double voltageLeft;
  private double voltageRight;
  private double voltageAngle;

=======
import edu.wpi.first.math.MathUtil;

public class ShooterSubsystem extends SubsystemBase {
  public ShooterIOReal IO = new ShooterIOReal();
>>>>>>> Stashed changes
  private double previousTime;
  private double previousRPS;

  private double acceleration;

  public ShooterSubsystem() {
<<<<<<< Updated upstream

    // plant = LinearSystemId.identifyVelocitySystem(Constants.ShooterConstants.flywheelKv, Constants.ShooterConstants.flywheelKa);
    // Vector<N1> Q = VecBuilder.fill(1.0/(2 * 2));
    // Vector<N1> R = VecBuilder.fill(1.0/(1 * 1));
    // controller = new LinearQuadraticRegulator<N1, N1, N1>(plant, Q, R, Constants.dt);
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
=======
  }
  public void spin(double velocity, double acceleration){
    double feedforward = Constants.ShooterConstants.flywheelKv * velocity + Constants.ShooterConstants.flywheelKa * acceleration + Math.signum(velocity) * Constants.ShooterConstants.flywheelKf;
    double feedback = (velocity - IO.rpsAvg()) * Constants.ShooterConstants.flywheelKp + acceleration * Constants.ShooterConstants.flywheelKd;
    double controlVoltage = feedforward + feedback;
    
    if(Math.abs(controlVoltage) > Constants.ShooterConstants.flywheelMax) controlVoltage = Math.signum(controlVoltage) * Constants.ShooterConstants.flywheelMax;
    IO.setShooterVoltage(controlVoltage);
  }
  public void setAnglePDF(double target_rad, double target_radPerSec){
    target_rad = MathUtil.clamp(target_rad, Constants.ShooterConstants.kLowerBound, Constants.ShooterConstants.kHigherBound);
    double error = target_rad - IO.angle();
    double voltagePosition = Constants.ShooterConstants.anglerKp * error + Constants.ShooterConstants.anglerKd * IO.rpsAngle();
    double voltageVelocity = Constants.ShooterConstants.anglerKv * target_radPerSec + Constants.ShooterConstants.anglerKvp * (target_radPerSec - IO.rpsAngle());
    //Friction correction applies when outside tolerance
    double frictionTolerance = 1 * Math.PI / 180;
    if(Math.abs(error) > frictionTolerance) voltagePosition += Constants.ShooterConstants.anglerKf * Math.signum(error);
    IO.setAnglerVoltage(voltagePosition + voltageVelocity);
>>>>>>> Stashed changes
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
<<<<<<< Updated upstream
  public Command voltageCommand(double voltageLeft, double voltageRight) {
=======
  public Command shooterVoltage(double voltageLeft, double voltageRight) {
>>>>>>> Stashed changes
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
<<<<<<< Updated upstream
          setShooterVoltage(voltageLeft, voltageRight);
=======
          IO.setShooterVoltage(voltageLeft);
>>>>>>> Stashed changes
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double curTime = Timer.getFPGATimestamp();

<<<<<<< Updated upstream
    acceleration = (rpsAvg() - previousRPS)/(curTime - previousTime);

    reportNumber("RPM/Left", rpsLeft() * 60);
    reportNumber("RPM/Right", rpsRight() * 60);
    reportNumber("ACC", acceleration);
    reportNumber("Voltage/Left", voltageLeft());
    reportNumber("Voltage/Right", voltageRight());

    previousTime = Timer.getFPGATimestamp();
    previousRPS = rpsAvg();
=======
    acceleration = (IO.rpsAvg() - previousRPS)/(curTime - previousTime);

    reportNumber("RPM/Left", IO.rpsLeft() * 60);
    reportNumber("RPM/Right",IO.rpsRight() * 60);
    reportNumber("ACC", acceleration);
    reportNumber("Voltage/Left", IO.voltageLeft());
    reportNumber("Voltage/Right", IO.voltageRight());

    previousTime = Timer.getFPGATimestamp();
    previousRPS = IO.rpsAvg();
    
>>>>>>> Stashed changes
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
