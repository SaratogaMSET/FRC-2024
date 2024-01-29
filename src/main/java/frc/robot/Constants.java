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

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static class ShooterConstants{
    public static final int kLeftMotorPort = 26;
    public static final int kRightMotorPort = 50;
    public static final int kAngleMotorPort = 50; //TODO: change
    public static final int kFeederMotorPort = 2;//TODO: change

    public static final int kEncoderPort = 0;

    public static final double kEncoderOffset = 0;
    public static final double kLowerBound = -1;
    public static final double kHigherBound = 1;

    public static final double kP = 0.023;
    public static final double kD = 0.01;
    public static final double kF = 0.0;

    public static final double kFlywheelKv = 0.023;
    public static final double kFlywheelKa = 0.001;
    public static final double kFlywheelMax = 12;

    public static class Regression{
      public static final double flightTime = 0.5252273393042887;
      public static class Angle{
        public static final double a = 1.09075;
        public static final double b = 2.89717;
        public static final double h = -1.08676;
        public static final double k = -0.522812;
      }

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

    public static final double kP = 0.023;
    public static final double kI = 0;
    public static final double kD = 0.01;
    public static final double kF = 0.0;
  }
  public static final double dt = 0.02;
}
