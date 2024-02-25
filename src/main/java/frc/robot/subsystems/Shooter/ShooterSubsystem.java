// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import frc.robot.Constants.ShooterFlywheelConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Turret.TurretIO;
import frc.robot.subsystems.Turret.TurretIOInputsAutoLogged;
import frc.robot.subsystems.Turret.TurretIOReal;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ShooterAnglerConstants;
import frc.robot.Constants.ShooterFeederConstants;

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

  public TurretIO turretIO;
  public TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();

  public SimpleMotorFeedforward shooterFF;
  public SimpleMotorFeedforward turretFF;
  public SimpleMotorFeedforward pivotFF;

  public PIDController shooterPid; 
  public PIDController turretPid; 
  public PIDController pivotPid; 

  private double startTime;
  public ShooterVisualizer viz = new ShooterVisualizer(getSubsystem(), null, ()->turretDegrees(), ()->pivotDegrees());
  public ShooterSubsystem(ShooterIO shooterIO, TurretIO turretIO) {
    this.shooterIO = shooterIO;
    this.turretIO = turretIO;
    if(Robot.isReal()){
      shooterPid = new PIDController(ShooterFlywheelConstants.kP, 0.0, ShooterFlywheelConstants.kD);
      turretPid = new PIDController(ShooterAnglerConstants.kP, 0.0, ShooterAnglerConstants.kD);
      pivotPid = new PIDController(TurretConstants.kP, 0.0, TurretConstants.kD);

      shooterFF = new SimpleMotorFeedforward(ShooterFlywheelConstants.kF, ShooterFlywheelConstants.kV, ShooterFlywheelConstants.kA);
      turretFF = new SimpleMotorFeedforward(TurretConstants.kF, TurretConstants.kV);
      pivotFF = new SimpleMotorFeedforward(ShooterAnglerConstants.kF, ShooterAnglerConstants.kV);
    }
    if(Robot.isSimulation()){
      shooterPid = new PIDController(ShooterFlywheelConstants.Sim.kP, 0.0, ShooterFlywheelConstants.Sim.kD);
      turretPid = new PIDController(ShooterAnglerConstants.Sim.kP, 0.0, ShooterAnglerConstants.Sim.kD);
      pivotPid = new PIDController(TurretConstants.Sim.kP, 0.0, TurretConstants.Sim.kD);

      shooterFF = new SimpleMotorFeedforward(ShooterFlywheelConstants.Sim.kF, ShooterFlywheelConstants.Sim.kV, ShooterFlywheelConstants.Sim.kA);
      turretFF = new SimpleMotorFeedforward(TurretConstants.Sim.kF, TurretConstants.Sim.kV);
      pivotFF = new SimpleMotorFeedforward(ShooterAnglerConstants.Sim.kF, ShooterAnglerConstants.Sim.kV);
    }
    this.startTime = Timer.getFPGATimestamp();

    testCalculations();
  }

  public double pivotRad(){
    return shooterInputs.pivotRad; 
  }
  public double pivotDegrees(){
    return shooterInputs.pivotRad * 180 / Math.PI;
  }
  public double pivotRadPerSec(){
      return shooterInputs.pivotRadPerSec;
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
  public double voltagePivot(){
      return shooterInputs.pivotAppliedVolts;
  }
  public boolean isShooterRunning() {
      return Math.abs(shooterInputs.shooterRPS[0]) + Math.abs(shooterInputs.shooterRPS[1]) > 0.1;
  }
  public double turretVoltage(){
    return turretInputs.turretVoltage;
  } 
  //TODO: Calibrate Zero Positions
  public double turretRad(){
      return turretInputs.turretRad; 
  }
  public double turretDegrees(){
      return turretInputs.turretRad * 180 / Math.PI;
    }
  //TOOD: Add gear ratio
  public double turretRadPerSec(){
      return turretInputs.turretRadPerSec;
  }
  public boolean isTurretRunning() {
    // Query some boolean state, such as a digital sensor.
    return Math.abs(turretRadPerSec()) > 0.01;
  }
  public double[] maxAngleFromShooter(double shooterAngle){
    return new double[]{Constants.TurretConstants.kLowerBound, Constants.TurretConstants.kHigherBound}; //TODO: DEPENDENCY REGRESSION FROM SHOOTER ANGLE
  }
  public boolean[] speedCompensatedBoundsShooter(){
    double projection = pivotRad() + pivotRadPerSec() * 0.1;
    return new boolean[]{projection < ShooterAnglerConstants.kLowerBound, projection > ShooterAnglerConstants.kHigherBound};
  }
  public boolean[] speedCompensatedBoundsTurret(){
    double projection = turretRad() + turretRadPerSec() * 0.1;
    return new boolean[]{projection < TurretConstants.kLowerBound, projection > TurretConstants.kHigherBound}; //TODO: DEPENDENCY REGRESSION FROM SHOOTER ANGLE
  }

  public void setShooterVoltage(double voltage){
    shooterIO.setShooterVoltage(voltage);
  }
  public void setPivotVoltage(double voltage){
    //TODO: Factor in velocity, if velocity will hit it in N control iterations, reduce by a factor based on how quickly it would hit based on current velocity
    //TODO: BOUNDS, FEEDFORWARD in NONPRIMITIVE
    boolean[] boundsTriggered = speedCompensatedBoundsShooter();
    if(boundsTriggered[0] && voltage < 0){
      voltage = 0;
    }

    if(boundsTriggered[1] && voltage > 0){
      voltage = 0;
    }

    shooterIO.setPivotVoltage(voltage);
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
    // double feedforward = ShooterFlywheelConstants.kV * targetRPM + ShooterFlywheelConstants.kA * acceleration + Math.signum(targetRPM) * ShooterFlywheelConstants.kF;
    // double feedback = (targetRPM - rpmShooterAvg()) * ShooterFlywheelConstants.kP + acceleration * ShooterFlywheelConstants.kD;

    double feedforward = shooterFF.calculate(targetRPM, acceleration);
    double feedback = shooterPid.calculate(rpmShooterAvg(), targetRPM);
    double controlVoltage = feedforward + feedback;
    
    if(Math.abs(controlVoltage) > ShooterFlywheelConstants.kVoltageMax) controlVoltage = Math.signum(controlVoltage) * ShooterFlywheelConstants.kVoltageMax;
    setShooterVoltage(controlVoltage);
  }
  public void setPivotPDF(double targetRad, double target_radPerSec){
    
    targetRad = MathUtil.clamp(targetRad, ShooterAnglerConstants.kLowerBound, ShooterAnglerConstants.kHigherBound);
    double error = targetRad - pivotRad();
    
    // double voltagePosition = ShooterAnglerConstants.kP * error + ShooterAnglerConstants.kD * pivotRadPerSec();
    double voltageVelocity = pivotFF.calculate(target_radPerSec) + ShooterAnglerConstants.kVP * (target_radPerSec - pivotRadPerSec());
    double voltagePosition = pivotPid.calculate(pivotRad(), targetRad);
    //Friction correction applies when outside tolerance
    double frictionTolerance = 1 * Math.PI / 180;
    if(Math.abs(error) > frictionTolerance) voltagePosition += ShooterAnglerConstants.kF * Math.signum(error);
    setPivotVoltage(voltagePosition + voltageVelocity);
  }
  public void setTurretPDF(double target_rad, double target_radPerSec){
    target_rad = MathUtil.clamp(target_rad, Constants.TurretConstants.kLowerBound, Constants.TurretConstants.kHigherBound);
    double error = target_rad - turretRad();
    
    // double voltagePosition = Constants.TurretConstants.kP * error + Constants.TurretConstants.kD * turretRadPerSec();
    // double voltageVelocity = Constants.TurretConstants.kV * target_radPerSec + Constants.TurretConstants.kVP * (target_radPerSec - turretRadPerSec());

    double voltagePosition = turretPid.calculate(turretRad(),target_rad);
    double voltageVelocity = turretFF.calculate(target_radPerSec) + Constants.TurretConstants.kVP * (target_radPerSec - turretRadPerSec());
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

  public void testCalculations(){
    ShooterCalculation shooterCalculation = new ShooterCalculation();
    double dDist = 0.01 * (Timer.getFPGATimestamp()-startTime);
    shooterCalculation.setState(5 + dDist, -2 - dDist, 0.1, 1, 1, 17);
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
    turretIO.updateInputs(turretInputs);
    Logger.processInputs("Shooter", shooterInputs);
    Logger.processInputs("Turret", turretInputs);

    reportNumber("Shooter RPM", rpmShooterAvg());
    reportNumber("Theta", pivotRad());
    reportNumber("Theta Speed", pivotRadPerSec() * 60/(2 * Math.PI));
    reportNumber("Voltage/Left", voltageShooterLeft());
    reportNumber("Voltage/Right", voltageShooterRight());

    reportNumber("Phi", turretDegrees());
    reportNumber("Phi Speed", turretRadPerSec() * 60);
    reportNumber("Voltage", turretVoltage());

    SmartDashboard.putBoolean("Shooter/Bounds/ShooterLow", speedCompensatedBoundsShooter()[0]);
    SmartDashboard.putBoolean("Shooter/Bounds/ShooterHigh", speedCompensatedBoundsShooter()[1]);
    SmartDashboard.putBoolean("Shooter/Bounds/TurretLow", speedCompensatedBoundsTurret()[0]);
    SmartDashboard.putBoolean("Shooter/Bounds/TurretHigh", speedCompensatedBoundsTurret()[1]);


    testCalculations();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    shooterIO.updateInputs(shooterInputs);    
    turretIO.updateInputs(turretInputs);
    Logger.processInputs("Shooter", shooterInputs);
    Logger.processInputs("Turret", turretInputs);
    viz.updateSim();
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
