package frc.robot.subsystems.Shooter.Angling;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.ShooterFlywheelConstants;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Turret.TurretIO;
import frc.robot.subsystems.Turret.TurretIOInputsAutoLogged;

public class AnglingSubsystem extends SubsystemBase{

    public SimpleMotorFeedforward turretFF;
    public SimpleMotorFeedforward pivotFF;

    public PIDController turretPid; 
    public PIDController pivotPid; 

    public TurretIO turretIO;
    public TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();
    
    public AnglingIO pivotIO;
    public AnglingIOInputsAutoLogged pivotInputs = new AnglingIOInputsAutoLogged();

    public AnglingSubsystem(AnglingIO pivotIO, TurretIO turretIO){
        this.turretIO = turretIO;
        this.pivotIO = pivotIO;
        if(Robot.isReal()){
            turretPid = new PIDController(TurretConstants.kP, 0.0, TurretConstants.kD);
            pivotPid = new PIDController(ShooterPivotConstants.kP, 0.0, ShooterPivotConstants.kD);

            turretFF = new SimpleMotorFeedforward(TurretConstants.kF, TurretConstants.kV);
            pivotFF = new SimpleMotorFeedforward(ShooterPivotConstants.kF, ShooterPivotConstants.kV);
        }
        if(Robot.isSimulation()){
            turretPid = new PIDController(ShooterPivotConstants.Sim.kP, 0.0, ShooterPivotConstants.Sim.kD);
            pivotPid = new PIDController(TurretConstants.Sim.kP, 0.0, TurretConstants.Sim.kD);
        }
        else{
            turretFF = new SimpleMotorFeedforward(TurretConstants.Sim.kF, TurretConstants.Sim.kV);
            pivotFF = new SimpleMotorFeedforward(ShooterPivotConstants.Sim.kF, ShooterPivotConstants.Sim.kV);
        }
    }

      public double pivotRad(){
        return pivotInputs.pivotRad; 
      }
      public double pivotDegrees(){
        return pivotInputs.pivotRad * 180 / Math.PI;
      }
      public double pivotRadPerSec(){
          return pivotInputs.pivotRadPerSec;
      }
      public double voltagePivot(){
        return pivotInputs.pivotAppliedVolts;
    }

     public void setPivotPDF(double targetRad, double target_radPerSec){
        if(speedCompensatedBoundsShooter(targetRad, target_radPerSec)[0] || speedCompensatedBoundsTurret(targetRad, target_radPerSec)[1]) target_radPerSec = 0;
        targetRad = MathUtil.clamp(targetRad, ShooterPivotConstants.kLowerBound, ShooterPivotConstants.kHigherBound);
        double error = targetRad - pivotRad();
        reportNumber("PivotError", error);
        // double voltagePosition = ShooterPivotConstants.kP * error + ShooterPivotConstants.kD * pivotRadPerSec();
        double voltageVelocity = pivotFF.calculate(target_radPerSec) + ShooterPivotConstants.kVP * (target_radPerSec - pivotRadPerSec());
        double voltagePosition = pivotPid.calculate(pivotRad(), targetRad);
        //Friction correction applies when outside tolerance
        double frictionTolerance = 1 * Math.PI / 180;
        if(Math.abs(error) > frictionTolerance) voltagePosition += ShooterPivotConstants.kF * Math.signum(error);
        reportNumber("PivotPosVolts", voltagePosition);
        setPivotVoltage(voltagePosition + voltageVelocity);
    }
    public double maxAbsTurretAngleFromPivot(){
        // -0.00383575 x^2 - 0.48873 x + 62.6542, where x is absolute value of pivotDegrees
        return Math.abs(-0.00383575 * pivotDegrees() * pivotDegrees() - 0.48873 *  pivotDegrees() + 60.6542) / 180 * Math.PI;
    }
   public boolean[] speedCompensatedBoundsTurret(){
    double turretBound = maxAbsTurretAngleFromPivot();
    SmartDashboard.putNumber("Shooter/Bounds/TurretBound", turretBound * 180 / Math.PI);
    double projection = turretRad() + turretRadPerSec() * 0.3;
    return new boolean[]{projection < -turretBound, projection > turretBound}; 
  }
  public boolean[] nonCompensatedBoundsTurret(){
    double turretBound = maxAbsTurretAngleFromPivot();
    double projection = turretRad();
    return new boolean[]{projection < -turretBound, projection > turretBound}; 
  }
  public boolean[] speedCompensatedBoundsTurret(double targetRad, double targetRadPerSec){
    double turretBound = maxAbsTurretAngleFromPivot();
    double projection = targetRad + targetRadPerSec * 0.3;
    return new boolean[]{projection < -turretBound, projection > turretBound};  
  }
  public void setPivotVoltage(double voltage){
    boolean[] compBoundsTriggered = speedCompensatedBoundsShooter();
    boolean[] nonCompBoundsTriggered = nonCompensatedBoundsShooter();
    if(compBoundsTriggered[0] && voltage < 0){
      if(nonCompBoundsTriggered[0]){
        voltage = 0;
      } 
      else{
        //voltage = Math.max(voltage * Math.max(Math.pow(pivotRad() - ShooterPivotConstants.kLowerBound, 2) * 1000, 1), 0); //Math.max(Math.pow(pivotRad() - ShooterPivotConstants.kLowerBound, 2) * 30000, 1);
      }
    }

    if(compBoundsTriggered[1] && voltage > 0){
      if(nonCompBoundsTriggered[1]){
        voltage = 0;
      } 
      else{
        //voltage = Math.max(voltage * Math.max(Math.pow(pivotRad() - ShooterPivotConstants.kHigherBound, 2) * 1000, 1), 0);//Math.max(Math.pow(pivotRad() - ShooterPivotConstants.kHigherBound, 2) * 30000, 1);
      }
    }

    pivotIO.setPivotVoltage(voltage);
  }

  public void setTurretVoltage(double voltage){
    //TODO: Tune RPS constant
    boolean[] boundsTriggered = nonCompensatedBoundsTurret();
    if(boundsTriggered[0] && voltage < 0){
        voltage = 0;
    }
    if(boundsTriggered[1] && voltage > 0){
        voltage = 0;
    }
    turretIO.setVoltage(voltage);
  }
  public void setTurretPDF(double target_rad, double target_radPerSec){
    if(speedCompensatedBoundsTurret(target_rad, target_radPerSec)[0] || speedCompensatedBoundsTurret(target_rad, target_radPerSec)[1]) target_radPerSec = 0;
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
  public Command pivotVoltage(double voltage) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          setPivotVoltage(voltage);
        });
  }
  public Command pivotAngleDegrees(double degrees) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          setPivotPDF(degrees /180 * Math.PI, 0);
        });
  }
  public Command turretVoltage(double voltage) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          setTurretVoltage(voltage);
        });
  }
  public Command turretAngleDegrees(double degrees) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          setTurretPDF(degrees / 180 * Math.PI, 0);
        });
  }
  public boolean[] speedCompensatedBoundsShooter(){
    double projection = pivotRad() + pivotRadPerSec() * 0.3;
    return new boolean[]{projection < ShooterPivotConstants.kLowerBound, projection > ShooterPivotConstants.kHigherBound};
  }
  public boolean[] nonCompensatedBoundsShooter(){
    double projection = pivotRad();
    return new boolean[]{projection < ShooterPivotConstants.kLowerBound, projection > ShooterPivotConstants.kHigherBound};
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
  public boolean[] speedCompensatedBoundsShooter(double targetRad, double targetRadPerSec){
    double projection = targetRad + targetRadPerSec * 0.3;
    return new boolean[]{projection < ShooterPivotConstants.kLowerBound, projection > ShooterPivotConstants.kHigherBound};
  }

  @Override 
  public void periodic(){

    pivotIO.updateInputs(pivotInputs);
    turretIO.updateInputs(turretInputs);
    Logger.processInputs("Turret", turretInputs);
    Logger.processInputs("Pivot", pivotInputs);

    reportNumber("Theta Deg", pivotRad() * 180/Math.PI);
    reportNumber("Theta Speed", pivotRadPerSec() * 180/Math.PI);
    reportNumber("Phi Deg", turretRad() * 180/Math.PI);
    reportNumber("Phi Speed", turretRadPerSec() * 180/Math.PI);
    reportNumber("Voltage", turretVoltage());


    SmartDashboard.putBoolean("Shooter/Bounds/ShooterLow", speedCompensatedBoundsShooter()[0]);
    SmartDashboard.putBoolean("Shooter/Bounds/ShooterHigh", speedCompensatedBoundsShooter()[1]);
    SmartDashboard.putBoolean("Shooter/Bounds/TurretLow", speedCompensatedBoundsTurret()[0]);
    SmartDashboard.putBoolean("Shooter/Bounds/TurretHigh", speedCompensatedBoundsTurret()[1]);
  }

  public void reportNumber(String name, double number){
    String prefix = "Angling/";
    SmartDashboard.putNumber(prefix + name, number);
  }
}
