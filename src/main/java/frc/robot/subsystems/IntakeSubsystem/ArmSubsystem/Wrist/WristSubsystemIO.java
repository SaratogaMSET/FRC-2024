package frc.robot.subsystems.IntakeSubsystem.ArmSubsystem.Wrist;
import frc.robot.Constants.IntakeSubsystem.Arm.ArmState;
import org.littletonrobotics.junction.AutoLog;

public interface WristSubsystemIO {

    @AutoLog
    public static class WristSubsystemIOInputs{
        public double wristDegrees = 0.0;
        public double wristCurrent = 0.0;
        public double wristVoltage = 0.0;
    }
    /**
   * Update the given inputs
   * @param inputs
   */
    public default void updateInputs(WristSubsystemIOInputs ioInputs){}

    /**
   * @param angle in degrees, where the horizontal is 0
   * @param velocity within the interval [0, 1]
   */
    public default void setAngle(double angle, double speedMagnitude){}

    /**
   * @param voltage within the interval [0, 1]
   */
    public default void setVoltage(double voltage){}


}
