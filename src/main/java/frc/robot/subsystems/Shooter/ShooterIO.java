package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {
    public double[] shooterRPS = {0.0, 0.0};

    public double pivotRad = 0;
    public double pivotRadPerSec = 0;

    public double[] shooterAppliedVolts = {0.0, 0.0};
    public double[] shooterAppliedCurrent = {0.0, 0.0};

    public double pivotAppliedVolts = 0.0;
    public double pivotAppliedCurrent = 0.0;

    public double feederAppliedVolts = 0.0;
    public double feederAppliedCurrent = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setDesiredRPM(double RPM) {}

  public default void setPivotProfiled(double target, double additionalVoltage) {}

  public default void setShooterVoltage(double voltage) {}

  public default void setPivotVoltage(double voltage) {}

  public default void setFeederVoltage(double voltage) {}

  public default void resetPivotEncoder() {}

  public default void setBeamBreak(boolean isTriggered) {}
}
