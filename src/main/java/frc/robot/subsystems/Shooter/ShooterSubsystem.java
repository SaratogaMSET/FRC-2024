// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterFlywheelConstants;
import frc.robot.Constants.ShooterPivotConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Robot;
import frc.robot.commands.Shooter.AimTestCommand;
import frc.robot.subsystems.Turret.TurretIO;
import frc.robot.subsystems.Turret.TurretIOInputsAutoLogged;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

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
  private double targetRPM;
  public ShooterVisualizer viz =
      new ShooterVisualizer(getSubsystem(), null, this::turretDegrees, this::pivotDegrees);

  ShooterCalculation solver = new ShooterCalculation();
  boolean previouslyInZone = false;

  public ShooterSubsystem(ShooterIO shooterIO, TurretIO turretIO) {
    this.shooterIO = shooterIO;
    this.turretIO = turretIO;
    if (Robot.isReal()) {
      shooterPid = new PIDController(ShooterFlywheelConstants.kP, 0.0, ShooterFlywheelConstants.kD);
      turretPid = new PIDController(TurretConstants.kP, 0.0, TurretConstants.kD);
      pivotPid = new PIDController(ShooterPivotConstants.kP, 0.0, ShooterPivotConstants.kD);

      pivotPid.setTolerance(0.4); // Radians?

      shooterFF =
          new SimpleMotorFeedforward(
              ShooterFlywheelConstants.kF,
              ShooterFlywheelConstants.kV,
              ShooterFlywheelConstants.kA);
      turretFF = new SimpleMotorFeedforward(0, TurretConstants.kV);
      pivotFF = new SimpleMotorFeedforward(0, ShooterPivotConstants.kV);
    }
    if (Robot.isSimulation()) {
      shooterPid =
          new PIDController(ShooterFlywheelConstants.Sim.kP, 0.0, ShooterFlywheelConstants.Sim.kD);
      turretPid =
          new PIDController(ShooterPivotConstants.Sim.kP, 0.0, ShooterPivotConstants.Sim.kD);
      pivotPid = new PIDController(TurretConstants.Sim.kP, 0.0, TurretConstants.Sim.kD);

      shooterFF =
          new SimpleMotorFeedforward(
              ShooterFlywheelConstants.Sim.kF,
              ShooterFlywheelConstants.Sim.kV,
              ShooterFlywheelConstants.Sim.kA);
      turretFF = new SimpleMotorFeedforward(0, TurretConstants.Sim.kV);
      pivotFF = new SimpleMotorFeedforward(0, ShooterPivotConstants.Sim.kV);
    }
    this.startTime = Timer.getFPGATimestamp();
    testCalculations();
  }

  public double pivotRad() {
    return shooterInputs.pivotRad;
  }

  public double pivotDegrees() {
    return shooterInputs.pivotRad * 180 / Math.PI;
  }

  public double pivotRadPerSec() {
    return shooterInputs.pivotRadPerSec;
  }
  // TODO: motor RPS vs output RPS, if geared
  public double rpsShooterAvg() {
    return (shooterInputs.shooterRPS[0] + shooterInputs.shooterRPS[1]) / 2;
  }

  public double rpmShooterAvg() {
    return (shooterInputs.shooterRPS[0] + shooterInputs.shooterRPS[1]) * 30;
  }

  public double voltageShooterLeft() {
    return shooterInputs.shooterAppliedVolts[0];
  }

  public double voltageShooterRight() {
    return shooterInputs.shooterAppliedVolts[1];
  }

  public double voltagePivot() {
    return shooterInputs.pivotAppliedVolts;
  }

  public boolean isShooterRunning() {
    return Math.abs(shooterInputs.shooterRPS[0]) + Math.abs(shooterInputs.shooterRPS[1]) > 0.1;
  }

  public double turretVoltage() {
    return turretInputs.turretVoltage;
  }

  public double turretRad() {
    return turretInputs.turretRad;
  }

  public double turretDegrees() {
    return turretInputs.turretRad * 180 / Math.PI;
  }
  // TOOD: Add gear ratio
  public double turretRadPerSec() {
    return turretInputs.turretRadPerSec;
  }
  /* Return if the turret is currently moving  */
  public boolean isTurretRunning() {
    return Math.abs(turretRadPerSec()) > 0.01;
  }

  public double[] maxAngleFromShooter(double shooterAngle) {
    return new double[] {
      Constants.TurretConstants.kLowerBound, Constants.TurretConstants.kHigherBound
    }; // TODO: DEPENDENCY REGRESSION FROM SHOOTER ANGLE
  }

  public boolean[] speedCompensatedBoundsShooter() {
    double projection = pivotRad() + pivotRadPerSec() * 0.3;
    return new boolean[] {
      projection < ShooterPivotConstants.kLowerBound,
      projection > ShooterPivotConstants.kHigherBound
    };
  }

  public boolean[] nonCompensatedBoundsShooter() {
    double projection = pivotRad();
    return new boolean[] {
      projection < ShooterPivotConstants.kLowerBound,
      projection > ShooterPivotConstants.kHigherBound
    };
  }

  public boolean[] speedCompensatedBoundsShooter(double targetRad, double targetRadPerSec) {
    double projection = targetRad + targetRadPerSec * 0.3;
    return new boolean[] {
      projection < ShooterPivotConstants.kLowerBound,
      projection > ShooterPivotConstants.kHigherBound
    };
  }

  public double maxAbsTurretAngleFromPivot() {
    // -0.00383575 x^2 - 0.48873 x + 62.6542, where x is absolute value of pivotDegrees
    return Math.abs(
            -0.00383575 * pivotDegrees() * pivotDegrees() - 0.48873 * pivotDegrees() + 60.6542 - 3)
        / 180
        * Math.PI;
  }

  public boolean[] speedCompensatedBoundsTurret() {
    double turretBound = maxAbsTurretAngleFromPivot();
    SmartDashboard.putNumber("Shooter/Bounds/TurretBound", turretBound * 180 / Math.PI);
    double projection = turretRad() + turretRadPerSec() * 0.3;
    return new boolean[] {projection < -turretBound, projection > turretBound};
  }

  public boolean[] nonCompensatedBoundsTurret() {
    double turretBound = maxAbsTurretAngleFromPivot();
    double projection = turretRad();
    return new boolean[] {projection < -turretBound, projection > turretBound};
  }

  public boolean[] speedCompensatedBoundsTurret(double targetRad, double targetRadPerSec) {
    double turretBound = maxAbsTurretAngleFromPivot();
    double projection = targetRad + targetRadPerSec * 0.3;
    return new boolean[] {projection < -turretBound, projection > turretBound};
  }

  public void setShooterVoltage(double voltage) {
    shooterIO.setShooterVoltage(voltage);
  }

  public void setPivotVoltage(double voltage) {
    boolean[] compBoundsTriggered = speedCompensatedBoundsShooter();
    boolean[] nonCompBoundsTriggered = nonCompensatedBoundsShooter();
    if (compBoundsTriggered[0] && voltage < 0) {
      if (nonCompBoundsTriggered[0]) {
        voltage = 0;
      } else {
        // voltage = Math.max(voltage * Math.max(Math.pow(pivotRad() -
        // ShooterPivotConstants.kLowerBound, 2) * 1000, 1), 0); //Math.max(Math.pow(pivotRad() -
        // ShooterPivotConstants.kLowerBound, 2) * 30000, 1);
      }
    }

    if (compBoundsTriggered[1] && voltage > 0) {
      if (nonCompBoundsTriggered[1]) {
        voltage = 0;
      } else {
        // voltage = Math.max(voltage * Math.max(Math.pow(pivotRad() -
        // ShooterPivotConstants.kHigherBound, 2) * 1000, 1), 0);//Math.max(Math.pow(pivotRad() -
        // ShooterPivotConstants.kHigherBound, 2) * 30000, 1);
      }
    }

    shooterIO.setPivotVoltage(voltage);
  }

  public void setFeederVoltage(double voltage) {
    shooterIO.setFeederVoltage(voltage);
  }

  public void setTurretVoltage(double voltage) {
    // TODO: Tune RPS constant
    boolean[] boundsTriggered = nonCompensatedBoundsTurret();
    if (boundsTriggered[0] && voltage < 0) {
      voltage = 0;
    }
    if (boundsTriggered[1] && voltage > 0) {
      voltage = 0;
    }
    turretIO.setVoltage(voltage);
  }

  public double getTargetRPM() {
    return targetRPM;
  }

  public void spinShooterMPS(double mps, double additionalRPM) {
    targetRPM = ShooterParameters.mps_to_kRPM(mps) * 1000 + additionalRPM;
    spinShooter(ShooterParameters.mps_to_kRPM(mps) * 1000 + additionalRPM);
  }

  public void spinShooter(double targetRPM) {
    double feedforward = ShooterParameters.kRPM_to_voltage(targetRPM / 1000);
    double feedback = (targetRPM - rpmShooterAvg()) * ShooterFlywheelConstants.kP;

    double controlVoltage = feedforward + feedback;

    if (Math.abs(controlVoltage) > ShooterFlywheelConstants.kVoltageMax)
      controlVoltage = Math.signum(controlVoltage) * ShooterFlywheelConstants.kVoltageMax;
    setShooterVoltage(controlVoltage);
  }

  public void setPivotProfiled(double targetRad, double target_radPerSec) {
    if (speedCompensatedBoundsShooter(targetRad, target_radPerSec)[0]
        || speedCompensatedBoundsShooter(targetRad, target_radPerSec)[1]) target_radPerSec = 0;
    targetRad =
        MathUtil.clamp(
            targetRad, ShooterPivotConstants.kLowerBound, ShooterPivotConstants.kHigherBound);
    Logger.recordOutput(
        "RealOutputs/Shooter/Pivot/CurrentRotations",
        Units.radiansToRotations(shooterInputs.pivotRad));
    shooterIO.setPivotProfiled(targetRad, target_radPerSec * ShooterPivotConstants.kV);
  }

  public void setTurretProfiled(double targetRad, double target_radPerSec) {
    if (speedCompensatedBoundsTurret(targetRad, target_radPerSec)[0]
        || speedCompensatedBoundsTurret(targetRad, target_radPerSec)[1]) target_radPerSec = 0;
    // For you sillies reading, the target is not clamped to anything speed compensated. check
    // "maxAbsTurretAngleFromPivot()"
    targetRad =
        MathUtil.clamp(targetRad, -maxAbsTurretAngleFromPivot(), maxAbsTurretAngleFromPivot());
    turretIO.setProfiled(targetRad, target_radPerSec * TurretConstants.kV);
  }

  public void setPivotPDF(double targetRad, double target_radPerSec) {
    if (speedCompensatedBoundsShooter(targetRad, target_radPerSec)[0]
        || speedCompensatedBoundsShooter(targetRad, target_radPerSec)[1]) target_radPerSec = 0;
    targetRad =
        MathUtil.clamp(
            targetRad, ShooterPivotConstants.kLowerBound, ShooterPivotConstants.kHigherBound);
    double error = targetRad - pivotRad();
    reportNumber("Pivot target rad", Units.degreesToRadians(targetRad));

    double voltagePosition =
        Constants.ShooterPivotConstants.kP * error
            - Constants.ShooterPivotConstants.kD * pivotRadPerSec();
    // double voltagePosition = pivotPid.calculate(pivotRad(), targetRad);
    double voltageVelocity = pivotFF.calculate(target_radPerSec);
    double voltageFriction = Math.signum(error) * 0.18;
    if (Math.abs(error) < 0.013) voltageFriction = 0;

    reportNumber("PivotPosVolts", voltagePosition);
    reportNumber("Pivot Position", Math.toDegrees(pivotRad()));
    double outputVolts = MathUtil.clamp(voltagePosition + voltageVelocity + voltageFriction, -4, 4);
    setPivotVoltage(outputVolts);
  }

  public void setTurretPDF(double target_rad, double target_radPerSec) {
    Logger.recordOutput("Turret Unclamped Setpoint", target_rad);
    if (speedCompensatedBoundsTurret(target_rad, target_radPerSec)[0]
        || speedCompensatedBoundsTurret(target_rad, target_radPerSec)[1]) target_radPerSec = 0;
    target_rad =
        MathUtil.clamp(target_rad, -maxAbsTurretAngleFromPivot(), maxAbsTurretAngleFromPivot());
    double error = target_rad - turretRad();
    reportNumber("Turret target rad", Math.toDegrees(target_rad));
    reportNumber("Turret Position", Math.toDegrees(turretRad()));
    double voltagePosition =
        Constants.TurretConstants.kP * error - Constants.TurretConstants.kD * turretRadPerSec();
    // double voltageVelocity = Constants.TurretConstants.kV * target_radPerSec +
    // Constants.TurretConstants.kVP * (target_radPerSec - turretRadPerSec());

    // double voltagePosition = turretPid.calculate(turretRad(),target_rad);
    double voltageVelocity = turretFF.calculate(target_radPerSec);
    double voltageFriction = Math.signum(error) * 0.13;
    if (Math.abs(error) < 0.013) voltageFriction = 0;

    double outputVolts = MathUtil.clamp(voltagePosition + voltageVelocity + voltageFriction, -4, 4);

    setTurretVoltage(outputVolts);
  }
  // CNUPIyellingPDF
  /**
   * Uses PDF (not I) to command the shooter to travel to a specific state
   *
   * @param shootVoltage the voltage to assign to the flywheels
   * @param turretAngleDegrees the angle to assign to the turret
   * @param pivotAngleDegrees the angle to assign to the pivot
   */
  public Command setShooterState(
      double shootVoltage, double turretAngleDegrees, double pivotAngleDegrees) {
    return setShooterStateRadians(
        shootVoltage, Math.toRadians(turretAngleDegrees), Math.toRadians(pivotAngleDegrees));
  }

  /**
   * Uses PDF (not I) to command the shooter to travel to a specific state
   *
   * @param shootVoltage desired MPS of note
   * @param turretAngleDegrees the angle to assign to the turret
   * @param pivotAngleDegrees the angle to assign to the pivot
   */
  public Command setShooterStateRadians(
      double shootVoltage, double turretAngleRadians, double pivotAngleRadians) {
    return this.run(
        () -> {
          setShooterVoltage(shootVoltage);
          setPivotProfiled(pivotAngleRadians, 0.0);
          setTurretProfiled(turretAngleRadians, 0.0);
        });
  }

  /**
   * Uses PDF (not I) to command the shooter to travel to a specific state
   *
   * @param shotMPS desired MPS of note
   * @param turretAngleDegrees the angle to assign to the turret
   * @param pivotAngleDegrees the angle to assign to the pivot
   */
  public Command setShooterStateMPS(
      double shotMPS, double turretAngleDegrees, double pivotAngleDegrees) {
    return setShooterStateMPS(shotMPS, 0, turretAngleDegrees, 0, pivotAngleDegrees, 0);
  }

  /**
   * Uses PDF (not I) to command the shooter to travel to a specific state
   *
   * @param shotMPS desired MPS of note
   * @param turretAngleDegrees the angle to assign to the turret
   * @param turretVelcityDegrees velocity for SOTM
   * @param pivotAngleDegrees the angle to assign to the pivot
   * @param pivotVelocityDegrees velocity for SOTM
   */
  public Command setShooterStateMPS(
      double shotMPS,
      double additionalRPM,
      double turretAngleDegrees,
      double turretVelocityDegrees,
      double pivotAngleDegrees,
      double pivotVelocityDegrees) {
    return setShooterStateMPSRadians(
        shotMPS,
        additionalRPM,
        Math.toRadians(turretAngleDegrees),
        Math.toRadians(turretVelocityDegrees),
        Math.toRadians(pivotAngleDegrees),
        Math.toRadians(pivotVelocityDegrees));
  }

  /** Uses PDF (not I) to command the shooter to travel to a specific state */
  public Command setShooterStateMPSRadians(
      double shotMPS,
      double additionalRPM,
      double turretAngleRadians,
      double turretVelocityRadians,
      double pivotAngleRadians,
      double pivotVelocityRadians) {
    return this.run(
        () -> {
          spinShooterMPS(shotMPS, additionalRPM);
          setPivotProfiled(pivotAngleRadians, pivotVelocityRadians);
          setTurretProfiled(turretAngleRadians, turretVelocityRadians);
        });
  }

  public Command shooterVoltage(double voltageShooter, double voltageFeeder) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setShooterVoltage(voltageShooter);
          setFeederVoltage(voltageFeeder);
        });
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
          setPivotProfiled(degrees / 180 * Math.PI, 0);
        });
  }

  public Command anglingDegrees(double turretDegrees, double pivotDegrees) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          setTurretProfiled(turretDegrees / 180 * Math.PI, 0.0);
          setPivotProfiled(pivotDegrees / 180 * Math.PI, 0);
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
          setTurretProfiled(degrees / 180 * Math.PI, 0);
        });
  }

  public boolean shooterReady() {
    return rpmShooterAvg() > getTargetRPM() - 100;
  }

  public Command aimTestCommandFactory(
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> robotSpeeds,
      boolean compensateGyro,
      double vMag,
      boolean shootSpeaker,
      boolean teleop,
      boolean autoShootInTeleop,
      double additionalRPM) {
    return new AimTestCommand(
        this,
        robotPose,
        robotSpeeds,
        compensateGyro,
        vMag,
        shootSpeaker,
        teleop,
        autoShootInTeleop,
        additionalRPM);
  }

  public void testCalculations() {
    ShooterCalculation shooterCalculation = new ShooterCalculation();
    double dDist = 0.01 * (Timer.getFPGATimestamp() - startTime);
    shooterCalculation.setState(5 + dDist, -2 - dDist, 0.1, 0, 1, 1, 17);
    double coldStartTime = Timer.getFPGATimestamp();
    double[] cold = shooterCalculation.solveAll(true, false);
    double warmStartTime = Timer.getFPGATimestamp();
    double[] warm = shooterCalculation.solveWarmStart(cold[0], cold[1], cold[2], true, false);
    // System.out.println("Cold Solve Time" + (warmStartTime - coldStartTime));
    // System.out.println("Warm Solve Time" + (Timer.getFPGATimestamp() - warmStartTime));
    // System.out.println("Phi: " + cold[0] + " Theta:" + cold[1] + "t:" + cold[2] + "dPhi" +
    // cold[3] + "dTheta" + cold[4]);
    reportNumber("Test/Cold/SolveTime", (warmStartTime - coldStartTime));
    reportNumber("Test/Warm/SolveTime", (Timer.getFPGATimestamp() - warmStartTime));

    reportNumber("Test/Cold/Params/Phi", cold[0] * 180 / Math.PI);
    reportNumber("Test/Cold/Params/Theta", cold[1] * 180 / Math.PI);
    reportNumber("Test/Cold/Params/T", cold[2]);
    reportNumber("Test/Cold/Params/dPhi", cold[3] * 180 / Math.PI);
    reportNumber("Test/Cold/Params/dTheta", cold[4] * 180 / Math.PI);

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

    if (this.getCurrentCommand() != null)
      Logger.recordOutput("ShooterCurrentCommand", this.getCurrentCommand().getName());

    reportNumber("Shooter RPM", rpmShooterAvg());
    reportNumber("Theta Deg", pivotRad() * 180 / Math.PI);
    reportNumber("Theta Speed", pivotRadPerSec() * 180 / Math.PI);
    reportNumber("Voltage/Left", voltageShooterLeft());
    reportNumber("Voltage/Right", voltageShooterRight());

    reportNumber("Phi Deg", turretRad() * 180 / Math.PI);
    reportNumber("Phi Speed", turretRadPerSec() * 180 / Math.PI);
    reportNumber("Voltage", turretVoltage());

    SmartDashboard.putBoolean("Shooter/Bounds/ShooterLow", speedCompensatedBoundsShooter()[0]);
    SmartDashboard.putBoolean("Shooter/Bounds/ShooterHigh", speedCompensatedBoundsShooter()[1]);
    SmartDashboard.putBoolean("Shooter/Bounds/TurretLow", speedCompensatedBoundsTurret()[0]);
    SmartDashboard.putBoolean("Shooter/Bounds/TurretHigh", speedCompensatedBoundsTurret()[1]);

    // testCalculations();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    shooterIO.updateInputs(shooterInputs);
    turretIO.updateInputs(turretInputs);
    Logger.processInputs("Shooter", shooterInputs);
    Logger.processInputs("Turret", turretInputs);
    viz.updateSim();

    if (this.getCurrentCommand() != null)
      Logger.recordOutput("ShooterCurrentCommand", this.getCurrentCommand().getName());
  }

  @Override
  public String getName() {
    return "Shooter Subsystem";
  }

  public void reportNumber(String name, double number) {
    String prefix = "Shooter/";
    Logger.recordOutput(prefix + name, number);
  }
}
