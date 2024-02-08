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
  public static class ClimbConstants{
    public static final int CLIMB_RIGHT_MOTOR = 20;
    public static final int CLIMB_LEFT_MOTOR = 21;
    public static final int HALLEFFECT = 22;
    public static final double SOFT_LIMIT_HEIGHT = 0.95; //max is 1.02235

    public static final double kP = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    public static final double kS = 0.0;
    public static final double kG = 0.0;

    public static final double carriageMassKg = 5.443;
    public static final double drumRadiusMeters = 0.0381;
    public static final double gearing = 25.0;

    public static class Sim{
      public static final double kP = 0.0;
      public static final double kD = 0.0;
      public static final double kV = 0.0;
      public static final double kA = 0.0;
       public static final double kS = 0.0;
      public static final double kG = 0.0;
    }
  }
}
