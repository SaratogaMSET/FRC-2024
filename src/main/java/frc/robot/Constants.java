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
        MANUAL
      }

      public static class Sim {
        public static final double ELEVATOR_LENGTH = 1; // Arbitrary, change me once hardware is finalized
        public static final double ELEVATOR_ANGLE = 90;
        public static final double SHOULDER_LENGTH = 1; // Arbitrary, change me once hardware is finalized
        public static final double WRIST_ANGLE = 20; // Arbitrary, change me once hardware is finalized
        public static final double WRIST_LENGTH = 0.2; // Arbitrary, change me once hardware is finalized
        public static final double SHOULDER_ANGLE = 30; // Arbitrary, change me once hardware is finalized
        public static final double SPEED = 0.2; // Arbitrary, change me during testing
      }

      public static class GroundNeutralPerimeterConstants {
        public static final double LOWER_MOTION_WRIST_ANGLE = 46.9;
        public static final double UPPER_MOTION_WRIST_ANGLE = 64.9;
        public static final double LOWER_MOTION_SHOULDER_ANGLE = 84.6;
        public static final double UPPER_MOTION_SHOULDER_ANGLE = 60.4;
        public static final double SHOULDER_POWER_PERCENT = 61.65;
        public static final double WRIST_POWER_PERCENT = 0;
      }

      public static class AmpScoringPositions {
        public static final double AMP_WRIST_ANGLE = 30;
        public static final double AMP_SHOULDER_ANGLE = 20;
      }

      public static class TrapScoringPositions {
        public static final double TRAP_WRIST_ANGLE = 80;
        public static final double TRAP_SHOULDER_ANGLE = 45.37;
      }

      public static class SourceScoringPositions {
        public static final double SOURCE_WRIST_ANGLE = 3.14;
        public static final double SOURCE_SHOULDER_ANGLE = 60.92;
      }

      public static class ControlsConstants {
        public static final double k_G = 0;
        public static final double k_P = 1;
        public static final double k_D = 0.000;
        public static final double k_I = 0.000; 
      }

      // Arbitrary, change all once we have a robot
      public static final int INTAKE_SHOULDER_MOTOR = 26;
      public static final int INTAKE_WRIST_MOTOR = 2;
      public static final int INTAKE_SHOULDER_ENCODER = 3;
      public static final int INTAKE_WRIST_ENCODER = 10;
      public static final NeutralModeValue ARM_NEUTRAL_MODE = NeutralModeValue.Brake;
      public static final double WRIST_ENCODER_OFFSET = 0; // In rotations
      public static final double SHOULDER_ENCODER_OFFSET = 0; // In rotations
      public static final double WRIST_HIGH_BOUND = 20;
      public static final double WRIST_LOW_BOUND = 0;
      public static final double SHOULDER_HIGH_BOUND = 20;
      public static final double SHOULDER_LOW_BOUND = 0;
      public static final double SHOULDER_ENCODER_OFFSET_FROM_ZERO = 0; // In degrees from horizontal as zero (for gravity feedforward calculations)
      public static final double WRIST_ENCODER_OFFSET_FROM_ZERO = 0; // In degrees from horizontal as zero (for gravity feedforward calculations)
      public static final double SHOULDER_ERROR_TOLERANCE = 0; // In degrees
      public static final double WRIST_ERROR_TOLERANCE = 0; // In degrees
      public static final double NEUTRAL_VOLTAGE = 0.0;
      public static final IdleMode INTAKE_BRAKE_MODE = IdleMode.kBrake;
      public static final int WRIST_CURRENT_LIMIT = 20;
      public static final double SHOULDER_OVERCOME_STATIC_VOLTAGE = 0.0;
      public static final double STATIC_SPEED = 0.0;
      public static final double MAX_SHOULDER_SPEED = 1.0;
      public static final double MAX_WRIST_SPEED = 0.0;

      public static class WristPIDConstants {}

      public static class ShoulderPIDConstants {}
    }

    // Arbitrary, change me once we have a robot
    public static class Roller {
      public static final int ROLLER_MOTOR = 5;
      public static final int ENTER_IR_GATE = 6;
      public static final int EXIT_IR_GATE = 21;
      public static final NeutralModeValue ROLLER_NEUTRAL_MODE = NeutralModeValue.Brake;
      public static final double HOLD_SPEED = 0.0;
      public static final double NEUTRAL_SPEED = 0.1;
      public static final double ROLLING_SPEED = 0.3;
      public static final double NEUTRAL_VOLTAGE = 0.0;


      public static enum RollerState {
        OUTTAKE,
        NEUTRAL,
        HOLD,
        AMP_INTAKE,
        SPEAKER_INTAKE
      }
    }
  }
}
