// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.Swerve;

import static frc.robot.subsystems.Swerve.Module.WHEEL_RADIUS;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.util.Queue;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

public class ModuleIOKrakenFOC implements ModuleIO {
  // Hardware
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private CANcoder cancoder;
  // private final AnalogInput turnAbsoluteEncoder;
  private final Rotation2d absoluteEncoderOffset;

  // Status Signals
  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveSupplyCurrent;
  private final StatusSignal<Double> driveTorqueCurrent;

  private final StatusSignal<Double> turnPosition;
  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnSupplyCurrent;
  private final StatusSignal<Double> turnTorqueCurrent;

  // Odometry Queues
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;
  private final Queue<Double> timestampQueue;

  // Controller Configs
  private final TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration turnTalonConfig = new TalonFXConfiguration();
  private static final Executor brakeModeExecutor = Executors.newFixedThreadPool(8);

  // Control
  private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0);
  private final TorqueCurrentFOC currentControl = new TorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
      new VelocityTorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final PositionTorqueCurrentFOC positionControl =
      new PositionTorqueCurrentFOC(0).withUpdateFreqHz(0);
  private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0);

  private final VelocityVoltage drivePIDF = new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0).withUpdateFreqHz(0);
  private final PositionVoltage turnPIDF = new PositionVoltage(0.0).withEnableFOC(true).withSlot(0).withUpdateFreqHz(0);

  // Constants
  // Gear ratios for SDS MK4i L4, adjust as necessary
  private double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
  private final double TURN_GEAR_RATIO = 150.0 / 7.0;
  private final boolean isTurnMotorInverted = true;
  private final double odometryFrequency = 250;

  public ModuleIOKrakenFOC(int index) {
    switch (Constants.getRobot()) {
      case ROBOT_2024P:
        DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
        switch (index) {
          case 0: // Front Left
            driveTalon = new TalonFX(38);
            turnTalon = new TalonFX(33);
            cancoder = new CANcoder(47);
            absoluteEncoderOffset = Rotation2d.fromDegrees(100.496 - 20.107); // -102.88477+179.8793
            break;
          case 1: // Front Right
            driveTalon = new TalonFX(30);
            turnTalon = new TalonFX(31);
            cancoder = new CANcoder(48);
            absoluteEncoderOffset =
                Rotation2d.fromDegrees(113.620 - 180 - 5.813 + 10.975); // -61.6793+0.21328
            break;
          case 2: // Back Left
            driveTalon = new TalonFX(35);
            turnTalon = new TalonFX(34);
            cancoder = new CANcoder(43);
            absoluteEncoderOffset = Rotation2d.fromDegrees(-123.387 + 6.563); // -121.40156-0.93164
            break;
          case 3: // Back Right
            driveTalon = new TalonFX(36);
            turnTalon = new TalonFX(37);
            cancoder = new CANcoder(41);
            absoluteEncoderOffset =
                Rotation2d.fromDegrees(-108.813 + 180 + 4.775); // -107.86406+178.95234
            break;
          default:
            throw new RuntimeException("Invalid module index");
        }
        break;
      case ROBOT_2024C:
        DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
        switch (index) {
          case 0:
            driveTalon = new TalonFX(11);
            turnTalon = new TalonFX(12);
            cancoder = new CANcoder(13);
            absoluteEncoderOffset = Rotation2d.fromDegrees(-81.244); // 10.020+90+180
            break;
          case 1:
            driveTalon = new TalonFX(21);
            turnTalon = new TalonFX(22);
            cancoder = new CANcoder(23);
            absoluteEncoderOffset = Rotation2d.fromDegrees(60.469 + 90 + 180); // -112.901+179.97
            break;
          case 2:
            driveTalon = new TalonFX(31);
            turnTalon = new TalonFX(32);
            cancoder = new CANcoder(33);
            absoluteEncoderOffset = Rotation2d.fromDegrees(-149.422 + 90); // 41.216+0.439
            break;
          case 3:
            driveTalon = new TalonFX(41);
            turnTalon = new TalonFX(42);
            cancoder = new CANcoder(43);
            absoluteEncoderOffset = Rotation2d.fromDegrees(7.8 + 180); // -79.365+90+180
            break;
          default:
            throw new RuntimeException("Invalid module index");
        }
        break;
      default:
        DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
        switch (index) {
          case 0:
            driveTalon = new TalonFX(11);
            turnTalon = new TalonFX(12);
            cancoder = new CANcoder(13);
            absoluteEncoderOffset =
                Rotation2d.fromDegrees(10.020 + 90 + 180); // -165.893-179.571+360
            break;
          case 1:
            driveTalon = new TalonFX(21);
            turnTalon = new TalonFX(22);
            cancoder = new CANcoder(23);
            absoluteEncoderOffset = Rotation2d.fromDegrees(60.469 + 90 + 180); // -112.901+179.97
            break;
          case 2:
            driveTalon = new TalonFX(31);
            turnTalon = new TalonFX(32);
            cancoder = new CANcoder(33);
            absoluteEncoderOffset = Rotation2d.fromDegrees(-149.422 + 90); // 41.216+0.439
            break;
          case 3:
            driveTalon = new TalonFX(41);
            turnTalon = new TalonFX(42);
            cancoder = new CANcoder(43);
            absoluteEncoderOffset = Rotation2d.fromDegrees(-79.365 + 90 + 180); // 103.654-0.45+180
            break;
          default:
            throw new RuntimeException("Invalid module index");
        }
    }
    driveTalonConfig.CurrentLimits.StatorCurrentLimit = 80; // try 120 if this is still slow
    driveTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    /* Need to retune for different units */
    driveTalonConfig.Slot0.kS = 0.025432; // / WHEEL_RADIUS; // /WHEEL_RADIUS
    driveTalonConfig.Slot0.kV = 0.15; // / WHEEL_RADIUS;
    driveTalonConfig.Slot0.kA = 0.032298; // / WHEEL_RADIUS;
    driveTalonConfig.Slot0.kP = 0.32; // / WHEEL_RADIUS;
    driveTalonConfig.Slot0.kD = 0.0;

    driveTalon.getConfigurator().apply(driveTalonConfig);
    setDriveBrakeMode(true);

    /* Closed loop duty cycle */
    turnTalonConfig.CurrentLimits.StatorCurrentLimit = 40.0;
    turnTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnTalonConfig.Slot0.kS = 0.025432; // volts to move
    turnTalonConfig.Slot0.kV = 0.15; // volts/rotation per seconds
    turnTalonConfig.Slot0.kA = 0; // / WHEEL_RADIUS;
    turnTalonConfig.Slot0.kP = 0.32; // volts/ rotation offset
    turnTalonConfig.Slot0.kD = 0.0;
    turnTalon.getConfigurator().apply(turnTalonConfig);
    
    setTurnBrakeMode(true);

    cancoder.getConfigurator().apply(new CANcoderConfiguration()); // don't bother.

    // Config Motors
    //driveTalonConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    //driveTalonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    //driveTalonConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
    //driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//
    //turnTalonConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    //turnTalonConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
    turnTalonConfig.MotorOutput.Inverted =
        isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    turnTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Conversions affect getPosition()/setPosition() and getVelocity()
    driveTalonConfig.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;
    turnTalonConfig.Feedback.SensorToMechanismRatio = TURN_GEAR_RATIO;
    turnTalonConfig.ClosedLoopGeneral.ContinuousWrap = true;

    // Apply configs
    for (int i = 0; i < 4; i++) {
      boolean error = driveTalon.getConfigurator().apply(driveTalonConfig, 0.1) == StatusCode.OK;
      error = error && (turnTalon.getConfigurator().apply(turnTalonConfig, 0.1) == StatusCode.OK);
      if (!error) break;
    }

    // 250hz signals
    drivePosition = driveTalon.getPosition();
    turnPosition = turnTalon.getPosition();
    BaseStatusSignal.setUpdateFrequencyForAll(odometryFrequency, drivePosition, turnPosition);

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, drivePosition);
    turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnPosition);

    // Get signals and set update rate
    // 100hz signals
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveSupplyCurrent = driveTalon.getSupplyCurrent();
    driveTorqueCurrent = driveTalon.getTorqueCurrent();
    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnSupplyCurrent = turnTalon.getSupplyCurrent();
    turnTorqueCurrent = turnTalon.getTorqueCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        driveVelocity,
        driveAppliedVolts,
        driveSupplyCurrent,
        driveTorqueCurrent,
        turnVelocity,
        turnAppliedVolts,
        turnSupplyCurrent,
        turnTorqueCurrent);

    // Reset turn position to absolute encoder position
    // turnTalon.setPosition(turnAbsolutePosition.get().getRotations(), 1.0);

    // Optimize bus utilization
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.hasCurrentControl = true;
    inputs.driveMotorConnected =
        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveAppliedVolts,
                driveSupplyCurrent,
                driveTorqueCurrent)
            .isOK();
    inputs.turnMotorConnected =
        BaseStatusSignal.refreshAll(
                turnPosition, turnVelocity, turnAppliedVolts, turnSupplyCurrent, turnTorqueCurrent)
            .isOK();

    inputs.drivePositionRads = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadsPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
    inputs.driveTorqueCurrentAmps = driveTorqueCurrent.getValueAsDouble();

    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadsPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnSupplyCurrentAmps = turnSupplyCurrent.getValueAsDouble();
    inputs.turnTorqueCurrentAmps = turnTorqueCurrent.getValueAsDouble();

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsMeters =
        drivePositionQueue.stream()
            .mapToDouble(signalValue -> Units.rotationsToRadians(signalValue) * WHEEL_RADIUS)
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void runDriveVolts(double volts) {
    driveTalon.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void runTurnVolts(double volts) {
    turnTalon.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void runCharacterization(double input) {
    driveTalon.setControl(currentControl.withOutput(input));
  }

  @Override
  public void runDriveVelocitySetpoint(double velocityRadsPerSec, double feedForward) {
    driveTalon.setControl(
       //velocityTorqueCurrentFOC
        drivePIDF
            .withVelocity(Units.radiansToRotations(velocityRadsPerSec))
            .withFeedForward(feedForward));
  }

  @Override
  public void runTurnPositionSetpoint(double angleRads) {
    turnTalon.setControl(turnPIDF.withPosition(Units.radiansToRotations(angleRads))); //used to be positionControl
  }

  @Override
  public void setDrivePID(double kP, double kI, double kD) {
    driveTalonConfig.Slot0.kP = kP;
    driveTalonConfig.Slot0.kI = kI;
    driveTalonConfig.Slot0.kD = kD;
    driveTalon.getConfigurator().apply(driveTalonConfig, 0.01);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnTalonConfig.Slot0.kP = kP;
    turnTalonConfig.Slot0.kI = kI;
    turnTalonConfig.Slot0.kD = kD;
    turnTalon.getConfigurator().apply(turnTalonConfig, 0.01);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    brakeModeExecutor.execute(
        () -> {
          synchronized (driveTalonConfig) {
            driveTalonConfig.MotorOutput.NeutralMode =
                enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            driveTalon.getConfigurator().apply(driveTalonConfig, 0.25);
          }
        });
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    brakeModeExecutor.execute(
        () -> {
          synchronized (turnTalonConfig) {
            turnTalonConfig.MotorOutput.NeutralMode =
                enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            turnTalon.getConfigurator().apply(turnTalonConfig, 0.25);
          }
        });
  }

  @Override
  public void stop() {
    driveTalon.setControl(neutralControl);
    turnTalon.setControl(neutralControl);
  }
}
