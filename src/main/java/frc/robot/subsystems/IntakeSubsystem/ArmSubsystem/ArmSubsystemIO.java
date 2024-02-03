// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.IntakeSubsystem.ArmSubsystem;

import frc.robot.Constants.IntakeSubsystem.Arm.ArmState;
import org.littletonrobotics.junction.AutoLog;

public interface ArmSubsystemIO {
  @AutoLog
  public static class ArmSubsystemIOInputs {
    public double shoulderDegrees = 0.0;
    public double wristDegrees = 0.0;
    public double elevatorHeight = 0.0;
    public ArmState armState = ArmState.NEUTRAL;
  }

  /**
   * @return
   */
  public abstract void updateInputs(ArmSubsystemIOInputs ioInputs); // FIXME: Check if this is still an error after
  // WPILIB compilation

  /**
   * @return
   */
  public double shoulderGetRadians();

  /**
   * @return
   */
  public double shoulderGetDegrees();

  /**
   * @return
   */
  public double wristGetRadians();

  /**
   * @return
   */
  public double wristGetDegrees();

  /**
   * @return
   */
  public double shoulderGetCurrent();

  /**
   * @return
   */
  public double shoulderGetVoltage();

  /**
   * @param angle
   * @param powerPercent
   */
 // public void shoulderSetAngle(double angle, double powerPercent);

  /**
   * @param angle
   * @param powerPercent
   */
  public void wristSetAngle(double angle, double powerPercent);

  public void shoulderSetAngle(double angle, double powerPercent);

  /** */
  public void gravityCompensation();
}
