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
  }
}
