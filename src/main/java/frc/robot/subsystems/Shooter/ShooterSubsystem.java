// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import frc.robot.Constants.ShooterFeederConstants;
import frc.robot.Constants.ShooterFlywheelConstants;
import frc.robot.Constants.ShooterPivotConstants;
// import frc.robot.Constants.ShooterFeederConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Turret.TurretIO;
import frc.robot.subsystems.Turret.TurretIOInputsAutoLogged;
import frc.robot.Constants;
import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class ShooterSubsystem extends SubsystemBase {
  public ShooterIO shooterIO;
  public ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  public SimpleMotorFeedforward shooterFF;

  public PIDController shooterPid; 

  private double startTime;
  // public ShooterVisualizer viz = new ShooterVisualizer(getSubsystem(), null, ()->turretDegrees(), ()->pivotDegrees());

  public ShooterSubsystem(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
    if(Robot.isReal()){
      shooterPid = new PIDController(ShooterFlywheelConstants.kP, 0.0, ShooterFlywheelConstants.kD);
      shooterFF = new SimpleMotorFeedforward(ShooterFlywheelConstants.kF, ShooterFlywheelConstants.kV, ShooterFlywheelConstants.kA);
    }
    if(Robot.isSimulation()){
      shooterPid = new PIDController(ShooterFlywheelConstants.Sim.kP, 0.0, ShooterFlywheelConstants.Sim.kD);
      shooterFF = new SimpleMotorFeedforward(ShooterFlywheelConstants.Sim.kF, ShooterFlywheelConstants.Sim.kV, ShooterFlywheelConstants.Sim.kA);
    }
    this.startTime = Timer.getFPGATimestamp();

    testCalculations();
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
  public boolean isShooterRunning() {
      return Math.abs(shooterInputs.shooterRPS[0]) + Math.abs(shooterInputs.shooterRPS[1]) > 0.1;
  }

  public Command shooterPercent(double shooterPercent, double voltageFeeder){
    return runOnce(
        () -> {
          spinShooter(shooterPercent * ShooterFlywheelConstants.kShooterMaxRPM, 0);
        });
  }
  public Command shooterVoltage(double voltageShooter, double voltageFeeder) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setShooterVoltage(voltageShooter);
        });
  }

  public void setShooterVoltage(double voltage){
    shooterIO.setShooterVoltage(voltage);
  }
  
  // public void setFeederVoltage(double voltage){
  //   shooterIO.setFeederVoltage(voltage);
  // }
  
  public void spinShooter(double targetRPM, double acceleration){
    double feedforward = ShooterFlywheelConstants.kV * targetRPM + ShooterFlywheelConstants.kA * acceleration + Math.signum(targetRPM) * ShooterFlywheelConstants.kF;
    double feedback = (targetRPM - rpmShooterAvg()) * ShooterFlywheelConstants.kP + acceleration * ShooterFlywheelConstants.kD;

    // double feedforward = shooterFF.calculate(targetRPM, acceleration);
    // double feedback = shooterPid.calculate(rpmShooterAvg(), targetRPM);
    double controlVoltage = feedforward + feedback;
    
    if(Math.abs(controlVoltage) > ShooterFlywheelConstants.kVoltageMax) controlVoltage = Math.signum(controlVoltage) * ShooterFlywheelConstants.kVoltageMax;
    setShooterVoltage(controlVoltage);
  }


  public void testCalculations(){
    ShooterCalculation shooterCalculation = new ShooterCalculation();
    double dDist = 0.01 * (Timer.getFPGATimestamp()-startTime);
    shooterCalculation.setState(5 + dDist, -2 - dDist, 0.1, 0, 1, 1, 17);
    double coldStartTime = Timer.getFPGATimestamp();
    double[] cold = shooterCalculation.solveAll();
    double warmStartTime = Timer.getFPGATimestamp();
    double[] warm = shooterCalculation.solveWarmStart(cold[0], cold[1], cold[2]);
    // System.out.println("Cold Solve Time" + (warmStartTime - coldStartTime));
    // System.out.println("Warm Solve Time" + (Timer.getFPGATimestamp() - warmStartTime));
    // System.out.println("Phi: " + cold[0] + " Theta:" + cold[1] + "t:" + cold[2] + "dPhi" + cold[3] + "dTheta" + cold[4]);
    reportNumber("Test/Cold/SolveTime", (warmStartTime - coldStartTime));
    reportNumber("Test/Warm/SolveTime", (Timer.getFPGATimestamp() - warmStartTime));

    reportNumber("Test/Cold/Params/Phi", cold[0] * 180/Math.PI);
    reportNumber("Test/Cold/Params/Theta", cold[1] * 180/Math.PI);
    reportNumber("Test/Cold/Params/T", cold[2]);
    reportNumber("Test/Cold/Params/dPhi", cold[3] * 180/Math.PI);
    reportNumber("Test/Cold/Params/dTheta", cold[4] * 180/Math.PI);

    reportNumber("Test/Warm/Params/Phi", warm[0]);
    reportNumber("Test/Warm/Params/Theta", warm[1]);
    reportNumber("Test/Warm/Params/T", warm[2]);
    reportNumber("Test/Warm/Params/dPhi", warm[3]);
    reportNumber("Test/Warm/Params/dTheta", warm[4]);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterIO.updateInputs(shooterInputs);    
    Logger.processInputs("Shooter", shooterInputs);

    reportNumber("Shooter RPM", rpmShooterAvg());
    reportNumber("Voltage/Left", voltageShooterLeft());
    reportNumber("Voltage/Right", voltageShooterRight());


    //testCalculations();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    shooterIO.updateInputs(shooterInputs);    
    Logger.processInputs("Shooter", shooterInputs);
    // viz.updateSim();
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
