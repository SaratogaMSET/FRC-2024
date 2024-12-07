package frc.robot.subsystems.Superstructure.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double leftCarriagePos = 0.0;
    public double rightCarriagePos = 0.0;
    public double[] carriagePositionMeters = {0.0, 0.0};
    public double secondStagePositionMeters = 0.0;
    public double[] elevatorVelocityMetersPerSec = {0.0, 0.0};
    public double[] elevatorAppliedVolts = {0.0, 0.0};
    public double[] elevatorCurrentAmps = {0.0, 0.0};
    public boolean hallEffectTriggered = true;
    public boolean heightLimitTriggered = false;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setDesiredHeight(double desiredHeight) {}

  public default void leftSetVoltage(double voltage) {}

  public default void rightSetVoltage(double voltage) {}

  public default void setVoltage(double voltage) {}

  public default void resetLeftEncoder() {}

  public default void resetRightEncoder() {}
}
