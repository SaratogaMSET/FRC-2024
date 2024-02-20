package frc.robot;

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

  public static class ShooterFlywheelConstants{
    public static final int kLeftMotorPort = 26;
    public static final int kRightMotorPort = 50;
    public static final double kShooterGearing = 0;

    public static final int kBeamBreakPort = 0;

    public static final double kP = 0;
    public static final double kD = 0;
    public static final double kF = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kVoltageMax = 12;
  }

  public static class ShooterFeederConstants{
    public static final int kMotorPort = 2;//TODO: change

    public static final double feedVoltage = 0;
  }
  public static class ShooterAnglerConstants{
    public static final int kMotorPort = 50; //TODO: change ports
    public static final double kMotorGearing = 0;

    public static final int kEncoderPort = 0;
    public static final double kEncoderOffset = 0;

    public static final double kP = 0.023;
    public static final double kD = 0.01;
    public static final double kF = 0.0;
    public static final double kV = 0.0;
    public static final double kVP = 0.0;

    public static final double kLowerBound = 0;
    public static final double kHigherBound = 1;

    public static final double kNeutralDegrees = 0;
  }
  public static class TurretConstants{
    public static final int kMotorPort = 0;
    public static final double kMotorGearing = 0;

    public static final int kEncoderPort = 0;

    public static final double kEncoderOffset = 0;
    public static final double kLowerBound = -1;
    public static final double kHigherBound = 1;

    public static final double kGearing = 1; //TODO: Change
    public static final double kP = 0.023;
    public static final double kI = 0;
    public static final double kD = 0.01;
    public static final double kF = 0.0;

    public static final double kV = 0;
    public static final double kVP = 0;
  }
}
