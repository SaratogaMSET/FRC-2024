// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.Swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn motor controller, and
 * CANcoder
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using an analog encoder, copy from "ModuleIOSparkMax")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
  private TalonFX driveTalon;
  private TalonFX turnTalon;
  private CANcoder cancoder;

  private final Queue<Double> timestampQueue;

  private final StatusSignal<Double> drivePosition;
  private final Queue<Double> drivePositionQueue;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;

  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnPosition;
  private final Queue<Double> turnPositionQueue;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrent;

  // Gear ratios for SDS MK4i L4, adjust as necessary
  private double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);

  private final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final boolean isTurnMotorInverted = true;
  private Rotation2d absoluteEncoderOffset;
  private VoltageOut driveVoltage = new VoltageOut(0.0).withEnableFOC(true);
  private VoltageOut turnVoltage = new VoltageOut(0.0).withEnableFOC(true);

  // private final MotionMagicVelocityVoltage drivePIDF =
  // new MotionMagicVelocityVoltage(0.0).withEnableFOC(true);
  private final VelocityVoltage drivePIDF =
      new VelocityVoltage(0.0).withEnableFOC(true).withSlot(0);

  public ModuleIOTalonFX(int index) {
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

    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = 90; // try 120 if this is still slow
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    driveConfig.Feedback.SensorToMechanismRatio = (1 / DRIVE_GEAR_RATIO) * ((2 * Math.PI));
    // (DRIVE_GEAR_RATIO) * (1.0 / (Module.WHEEL_RADIUS * 2 * Math.PI));
    driveConfig.Slot0.kS = 0.025432; // / WHEEL_RADIUS; // /WHEEL_RADIUS
    driveConfig.Slot0.kV = 0.15; // / WHEEL_RADIUS;
    driveConfig.Slot0.kA = 0.032298; // / WHEEL_RADIUS;
    driveConfig.Slot0.kP = 0.32; // / WHEEL_RADIUS;
    driveConfig.Slot0.kD = 0.0;

    // driveConfig.MotionMagic.MotionMagicCruiseVelocity =
    // SwerveSubsystem.MAX_LINEAR_SPEED/WHEEL_RADIUS;
    // driveConfig.MotionMagic.MotionMagicAcceleration = 9.8/WHEEL_RADIUS;
    driveConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
    driveConfig.MotionMagic.MotionMagicAcceleration = 0;

    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    var turnConfig = new TalonFXConfiguration();
    turnConfig.CurrentLimits.StatorCurrentLimit = 30.0;
    turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    turnTalon.getConfigurator().apply(turnConfig);
    setTurnBrakeMode(true);

    cancoder.getConfigurator().apply(new CANcoderConfiguration());

    timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();

    drivePosition = driveTalon.getPosition();
    drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    turnAbsolutePosition = cancoder.getAbsolutePosition();
    turnPosition = turnTalon.getPosition();
    turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(turnTalon, turnTalon.getPosition());
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrent = turnTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Module.ODOMETRY_FREQUENCY, drivePosition, turnPosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        drivePosition,
        driveVelocity,
        driveAppliedVolts,
        driveCurrent,
        turnAbsolutePosition,
        turnPosition,
        turnVelocity,
        turnAppliedVolts,
        turnCurrent);

    inputs.drivePositionRad = (drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = (driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble()) // check
            .minus(absoluteEncoderOffset);

    inputs.turnPosition =
        Rotation2d.fromRotations(turnPosition.getValueAsDouble() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsToRadians(turnVelocity.getValueAsDouble()) / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnCurrentAmps = new double[] {turnCurrent.getValueAsDouble()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble((Double value) -> (value)).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / TURN_GEAR_RATIO))
            .toArray(Rotation2d[]::new);

    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveSetpoint(final double radiansPerSecond) {
    driveTalon.setControl(drivePIDF.withVelocity(radiansPerSecond));
  }
  // @Override
  // public void setDriveSetpoint(final double radiansPerSecond) {
  //   driveTalon.setControl(drivePIDF.withVelocity(radiansPerSecond));
  // }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(driveVoltage.withOutput(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(turnVoltage.withOutput(volts));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted =
        isTurnMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turnTalon.getConfigurator().apply(config);
  }

  @Override
  public void setDriveCurrentLimit(double currentLimit) {
    var driveConfig = new TalonFXConfiguration();
    driveConfig.CurrentLimits.StatorCurrentLimit = currentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveTalon.getConfigurator().apply(driveConfig);
  }
}
