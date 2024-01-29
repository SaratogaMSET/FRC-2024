// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class TurretSubsystem extends SubsystemBase {
  TalonFX m_motor;
  CANcoder encoder;

  private double voltage;

  public TurretSubsystem() {
    m_motor = new TalonFX(Constants.TurretConstants.kMotorPort);

    m_motor.setControl(new StaticBrake());
    encoder = new CANcoder(Constants.TurretConstants.kEncoderPort);
  }

  public void configMotors(){
    TalonFXConfiguration generalConfig = new TalonFXConfiguration();
    MotorOutputConfigs motorConfig = new MotorOutputConfigs();
    ClosedLoopRampsConfigs voltageRampConfig = new ClosedLoopRampsConfigs();
    CurrentLimitsConfigs currentLimitConfig = new CurrentLimitsConfigs();

    motorConfig.withPeakForwardDutyCycle(1);
    motorConfig.withPeakReverseDutyCycle(1);
    
    voltageRampConfig.withVoltageClosedLoopRampPeriod(0.5);

    currentLimitConfig.withStatorCurrentLimit(5);
    currentLimitConfig.withStatorCurrentLimitEnable(true);

    generalConfig.withMotorOutput(motorConfig);
    generalConfig.withClosedLoopRamps(voltageRampConfig);
    generalConfig.withCurrentLimits(currentLimitConfig);

    //leftConfig.withSoftwareLimitSwitch(null);

    m_motor.getConfigurator().apply(generalConfig);
    m_motor.setInverted(false); //TODO:fix
  }

  public double rps(){
    return m_motor.getVelocity().getValueAsDouble();
  }
  public double voltage(){
    return this.voltage;
  } 

  //TODO: Calibrate Zero Positions
  public double angle(){
    return encoder.getAbsolutePosition().getValueAsDouble() - Constants.TurretConstants.kEncoderOffset; 
  }
  public void setVoltage(double voltage){

    if(angle() < Constants.TurretConstants.kLowerBound && voltage < 0){
      voltage = 0;
    }
    if(angle() > Constants.TurretConstants.kHigherBound && voltage > 0){
      voltage = 0;
    }

    m_motor.setVoltage(voltage);

    this.voltage = voltage;
  }
  public boolean setAnglePDF(double target_degrees){
    double target = target_degrees * 180 / Math.PI;
    double error = target - angle();
    double voltage = Constants.TurretConstants.kP * error + Constants.TurretConstants.kD * rps();
    //Friction correction applies when outside tolerance
    double frictionTolerance = 1 * Math.PI / 180;
    if(Math.abs(error) > frictionTolerance) voltage += Constants.TurretConstants.kF * Math.signum(error);
    setVoltage(voltage);


    if(Math.abs(error) < frictionTolerance) return false;
    return true;
  }
  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command voltageCommand(double voltage) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setVoltage(voltage);
        });
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean isRunning() {
    // Query some boolean state, such as a digital sensor.
    return Math.abs(rps()) < 0.1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    reportNumber("RPM", rps() * 60);
    reportNumber("Voltage", voltage());
    SmartDashboard.putBoolean("Turret/Bound/Low", angle() < Constants.TurretConstants.kLowerBound);
    SmartDashboard.putBoolean("Turret/Bound/High", angle() < Constants.TurretConstants.kHigherBound);
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
