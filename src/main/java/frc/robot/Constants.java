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

  public static class IntakeSubsystem {
    public static class Arm {
      public static enum ArmState {
        GROUND_DEPLOY,
        NEUTRAL,
        AMP,
        SOURCE,
        TRAP,
      }

      public static class Sim {
        public static final double ELEVATOR_LENGTH = 0.0;
        public static final double ELEVATOR_ANGLE = 0.0;
        public static final double SHOULDER_LENGTH = 0.0;
        public static final double WRIST_ANGLE = 0.0;
        public static final double WRIST_LENGTH = 0.0;
        public static final double SHOULDER_ANGLE = 0.0;
      }

      public static class GroundNeutralPerimeterConstants {
        public static final double LOWER_MOTION_WRIST_ANGLE = 0;
        public static final double UPPER_MOTION_WRIST_ANGLE = 0;
        public static final double LOWER_MOTION_SHOULDER_ANGLE = 0;
        public static final double UPPER_MOTION_SHOULDER_ANGLE = 0;
        public static final double SHOULDER_POWER_PERCENT = 0;
        public static final double WRIST_POWER_PERCENT = 0;
      }

      public static class AmpScoringPositions {
        public static final double AMP_WRIST_ANGLE = 0.0;
        public static final double AMP_SHOULDER_ANGLE = 0.0;
      }

      public static class SourceScoringPositions {
        public static final double SOURCE_WRIST_ANGLE = 0.0;
        public static final double SOURCE_SHOULDER_ANGLE = 0.0;
      }

      public static final int INTAKE_SHOULDER_MOTOR = 1;
      public static final int INTAKE_WRIST_MOTOR = 2;
      public static final int INTAKE_SHOULDER_ENCODER = 3;
      public static final int INTAKE_WRIST_ENCODER = 10;
      public static final NeutralModeValue ARM_NEUTRAL_MODE = NeutralModeValue.Brake;
      public static final double WRIST_ENCODER_OFFSET = 0; // In rotations
      public static final double SHOULDER_ENCODER_OFFSET = 0; // In rotations
      public static final double WRIST_HIGH_BOUND = 0;
      public static final double WRIST_LOW_BOUND = 0;
      public static final double SHOULDER_HIGH_BOUND = 0;
      public static final double SHOULDER_LOW_BOUND = 0;
      public static final double SHOULDER_ENCODER_OFFSET_FROM_ZERO =
          0; // In degrees from horizontal as zero (for gravity
      // feedforward calculations)
      public static final double WRIST_ENCODER_OFFSET_FROM_ZERO =
          0; // In degrees from horizontal as zero (for gravity
      // feedforward calculations)
      public static final double SHOULDER_ERROR_TOLERANCE = 0; // In degrees
      public static final double WRIST_ERROR_TOLERANCE = 0; // In degrees
      public static final double NEUTRAL_VOLTAGE = 0.0;
      public static final IdleMode INTAKE_BRAKE_MODE = IdleMode.kBrake;

      public static class WristPIDConstants {}

      public static class ShoulderPIDConstants {}
    }

    public static class Roller {
      public static final int ROLLER_MOTOR = 5;
      public static final int ENTER_IR_GATE = 6;
      public static final int EXIT_IR_GATE = 21;
      public static final NeutralModeValue ROLLER_NEUTRAL_MODE = NeutralModeValue.Brake;
      public static final double HOLD_SPEED = 0.0;
      public static final double NEUTRAL_SPEED = 0.0;
      public static final double NEUTRAL_VOLTAGE = 0.0;
      public static final double ROLLING_SPEED = 0.0;
    }
  }
}
