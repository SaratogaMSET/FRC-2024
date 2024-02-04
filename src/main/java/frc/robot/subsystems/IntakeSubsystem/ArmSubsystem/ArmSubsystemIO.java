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
   * Update the given inputs
   * @param inputs
   */
  public abstract void updateInputs(ArmSubsystemIOInputs ioInputs);

  /**
   * @return the angle of the shoulder in radians, where the horizontal is 0
   */
  public double shoulderGetRadians();

  /**
   * @return the angle of the shoulder in degrees, where the horizontal is 0
   */
  public double shoulderGetDegrees();

  /**
   * @return the angle of the wrist in radians, where the horizontal is 0
   */
  public double wristGetRadians();

  /**
   * @return the angle of the wrist in degrees, where the horizontal is 0
   */
  public double wristGetDegrees();

  /**
   * @return the motor current draw of the shoulder
   */
  public double shoulderGetCurrent();

  /**
   * @return the motor voltage draw of the shoulder
   */
  public double shoulderGetVoltage();

  /**
   * @param angle in degrees, where the horizontal is 0
   * @param velocity within the interval [0, 1]
   */
  public void wristSetAngle(double angle, double speedMagnitude);

  /**
   * @param angle in degrees, where the horizontal is 0
   * @param velocity within the interval [0, 1]
   */
  public void shoulderSetAngle(double angle, double speedMagnitude);

  /** 
   * Updates the applied voltage for gravity compensation
  */
  public void gravityCompensation();
}
