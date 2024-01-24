package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

public class Constants {
    public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class IntakeSubsystem{
    public static class Actuator{
      public static final int INTAKE_ELBOW_MOTOR = 0;
      public static final int INTAKE_WRIST_MOTOR = 0;
      public static final int INTAKE_ELBOW_ENCODER = 0;
      public static final int INTAKE_WRIST_ENCODER = 0;
      public static final int IR_GATE = 0;
      public static final NeutralModeValue ACTUATOR_NEUTRAL_MODE = NeutralModeValue.Brake;
      public static final double WRIST_ENCODER_OFFSET = 0;  // In rotations
      public static final double ELBOW_ENCODER_OFFSET = 0;  // In rotations
      public static final double WRIST_HIGH_BOUND = 0;
      public static final double WRIST_LOW_BOUND = 0;
      public static final double ELBOW_HIGH_BOUND = 0;
      public static final double ELBOW_LOW_BOUND = 0;
      public static final double ELBOW_ENCODER_OFFSET_FROM_ZERO = 0;  // In degrees from horizontal as zero (for gravity feedforward calculations)
      public static final double WRIST_ENCODER_OFFSET_FROM_ZERO = 0;  // In degrees from horizontal as zero (for gravity feedforward calculations)
      public static final double ELBOW_ERROR_TOLERANCE = 0; // In degrees
      public static final double WRIST_ERROR_TOLERANCE = 0; // In degrees
      public static final double NEUTRAL_VOLTAGE = 0.0;
      public static final IdleMode INTAKE_BRAKE_MODE = IdleMode.kBrake;
      
      public static class WristPIDConstants {

      }

      public static class ElbowPIDConstants {
        
      }
    }
    public static class Roller{
      public static final int ROLLER_MOTOR = 0;
    }
  }
}
