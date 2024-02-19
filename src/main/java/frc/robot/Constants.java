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

  public static class ShooterConstants{
    public static final int kLeftMotorPort = 26;
    public static final int kRightMotorPort = 50;
    public static final int kAngleMotorPort = 50; //TODO: change ports
    public static final int kFeederMotorPort = 2;//TODO: change

    public static final int kEncoderPort = 0;
    public static final int kBeamBreakPort = 0;

    public static final double kEncoderOffset = 0;
    public static final double kLowerBound = -1;
    public static final double kHigherBound = 1;

    public static final double angleRatio = 0; //TODO: Change
    public static final double anglerKp = 0.023;
    public static final double anglerKd = 0.01;
    public static final double anglerKf = 0.0;
    public static final double anglerKv = 0.0;
    public static final double anglerKvp = 0.0;

    public static final double flywheelKp = 0;
    public static final double flywheelKd = 0;
    public static final double flywheelKv = 0.023;
    public static final double flywheelKa = 0.001;
    public static final double flywheelKf = 0;
    public static final double flywheelMax = 12;

    public static class Regression{
      public static final double velocityTolerance = 50/60.0;
      public static final double angleTolerance = 0.5;
    }
  }

  public static class TurretConstants{
    public static final int kMotorPort = 0;

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

  public static final double dt = 0.02;
}
