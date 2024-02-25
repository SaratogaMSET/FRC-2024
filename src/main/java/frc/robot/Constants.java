package frc.robot;

public class Constants {
    public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class ShooterFlywheelConstants{
    public static final int kLeftMotorPort = 51;
    public static final int kRightMotorPort = 52;
    public static final double kShooterGearing = 1.8;
    public static final double kShooterMaxRPM = 6380;

    public static final int kBeamBreakPort = 0;

    public static final double kP = 0;
    public static final double kD = 0;
    public static final double kF = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kVoltageMax = 12;
     public class Sim{
      public static final double kP = 0.5;
      public static final double kD = 0.01;
      public static final double kF = 0.01;
      public static final double kV = 0.0;
      public static final double kVP = 0.0;
      public static final double kA = 0;
    }
  }

  public static class ShooterFeederConstants{
    public static final int kMotorPort = 54;

    public static final double feedVoltage = 0;
  }
  public static class ShooterPivotConstants{
    public static final int kMotorPort = 55;
    public static final double kMotorGearing = 60.0/16.0 * 36.0/20.0 * 110.0/10.0;

    public static final int kEncoderPort = 56;
    public static final double kEncoderOffset = 0;

    public static final double kP = 0.023;
    public static final double kD = 0.01;
    public static final double kF = 0.0;
    public static final double kV = 0.0;
    public static final double kVP = 0.0;

    public static final double kLowerBound = 0;
    public static final double kHigherBound = 1;

    public static final double kNeutralDegrees = 0;

    public class Sim{
      public static final double kP = 12;
      public static final double kD = 0.01;
      public static final double kF = 0.0;
      public static final double kV = 0.0;
      public static final double kVP = 0.0;
    }
  }
  public static class TurretConstants{
    public static final int kMotorPort = 53;
    public static final double kMotorGearing = 60.0;

    public static final int kEncoderPort = 57;

    public static final double kEncoderOffset = 0;
    public static final double kLowerBound = -1;
    public static final double kHigherBound = 1;

    public static final double kGearing = 1; //TODO: Change
    public static final double kP = 0.023;
    public static final double kI = 0;
    public static final double kD = 0.00;
    public static final double kF = 0.0;

    public static final double kV = 0;
    public static final double kVP = 0;
    public class Sim{
      public static final double kP = 1;
      public static final double kD = 0.00;
      public static final double kF = 0.0;
      public static final double kV = 0.0;
      public static final double kVP = 0.0;
    }
  }
}
