package frc.robot;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

public class Constants {

    public static final Mode currentMode = Mode.REAL; // This doesn't do anything.
    public static RobotType robot = RobotType.ROBOT_2024C;
    public static String CANBus = "649-Robot-CANivore";

    public static boolean invalidRobotAlertSent = false;

    public static void setRobot(RobotType type){
      robot = type;
    }

    public static Mode getMode(){
      return currentMode;
    }

    public static RobotType getRobot() {
      if (RobotBase.isReal()) { // What is HAL? 
        if (robot == RobotType.ROBOT_SIMBOT) { // Invalid robot selected
          if (!invalidRobotAlertSent) {
            new Alert("Invalid robot selected, using competition robot as default.", AlertType.ERROR)
                .set(true);
            invalidRobotAlertSent = true;
          }
          return RobotType.ROBOT_2024C;
        } else {
          return robot;
        }
      } else {
        return robot;
      }
    }

    public static enum RobotType {
      ROBOT_2024C,
      ROBOT_2024P,
      ROBOT_SIMBOT;

      public static List<String> getStringArray() {
        return Arrays.stream(RobotType.values()).map(Enum::name).collect(Collectors.toList());
      }

      public static List<RobotType> getList(){
        return Arrays.stream(RobotType.values()).collect(Collectors.toList());
      }

    }

    public static enum Mode {
      /** Running on a real robot. */
      REAL,

      /** Running a physics simulator. */
      SIM,

      /** Replaying from a log file. */
      REPLAY
    }

    public static class Vision {
        //back left camera for jaws
        public static final Transform3d jawsCamera0 = new Transform3d(-Units.inchesToMeters(9.25), (SwerveSubsystem.TRACK_WIDTH_Y/2), Units.inchesToMeters(8.25),
              new Rotation3d(0.0, -Math.toRadians(17.5),Math.toRadians(210)));
        //back right camera for jaws
        public static final Transform3d jawsCamera1 = new Transform3d(new Translation3d((-Units.inchesToMeters(9.25)), (-SwerveSubsystem.TRACK_WIDTH_Y/2), Units.inchesToMeters(8.25)),
            new Rotation3d(0.0, -Math.toRadians(17.5),Math.toRadians(150)));

        public static final Transform3d turretCamera0 = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0.0, -Units.degreesToRadians(17.5), 0.0));

        public static final double distanceFactor = 0.5;

        public static final Transform3d robotToCam14 = new Transform3d(new Translation3d(Units.inchesToMeters(9.75), -Units.inchesToMeters(13.5), Units.inchesToMeters(10.5)),
            new Rotation3d(Math.toRadians(20),0,Math.toRadians(135))); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d robotToCam11 = new Transform3d(new Translation3d(-Units.inchesToMeters(1), -Units.inchesToMeters(13.5), Units.inchesToMeters(11.3)),
            new Rotation3d(Math.toRadians(20) ,0,Math.toRadians(45))); //Cam mounted facing forward, 3 forward of center,15 inches left of center, 17 up from center.
        public static final Matrix<N3, N1> stateSTD = VecBuilder.fill(0.25, 0.25, 0.01); //0.995
        public static final Matrix<N3, N1> visDataSTD = VecBuilder.fill(0.25, 0.25, 0.25); // This gets filtered by scaling std's and whatnot. 

        public static final double ALIGNMENT_ALLOWED_TOLERANCE_TRANSLATIONAL = 0.1; // meters
        public static final double ALIGNMENT_ALLOWED_TOLERANCE_ROTATIONAL = 0.122; // radians

        public static double xyCoeff = 0.01; // TODO: FIX, THESE VALUES ARE NOT TUNED
        public static double rotationCoeff = 0.01; // TODO: FIX, THESE VALUES ARE NOT TUNED
    } 

  public static class Intake {  // Waiting for hardware (Akash) with angles to return from Tahoe
    public static class DesiredStates {
      public static class Ground {
        public static final double LOWER_MOTION_WRIST_ANGLE = 0.0;
        public static final double LOWER_MOTION_SHOULDER_ANGLE = -0.69;//Math.toRadians(-36);
        public static final double SHOULDER_VELOCITY = 1;
        public static final double WRIST_VELOCITY = 1;
      }

      public static class Amp {
        public static final double WRIST_ANGLE = 1.5; //1
        public static final double SHOULDER_ANGLE = 1.35;
        public static final double elevatorPosition = 0.5; //TODO: TUNE
        public static final double SHOULDER_VELOCITY = 1;
        public static final double WRIST_VELOCITY = 1; 
      }

      public static class Trap {
        public static final double WRIST_ANGLE = 0.0;
        public static final double SHOULDER_ANGLE = 0.0;
        public static final double SHOULDER_VELOCITY = 1;
        public static final double WRIST_VELOCITY = 1;
      }

      public static class Source {
        public static final double WRIST_ANGLE = 0.0;
        public static final double SHOULDER_ANGLE = 0.0;
        public static final double SHOULDER_VELOCITY = 1;
        public static final double WRIST_VELOCITY = 1;
      }

      public static class Neutral {
        public static final double WRIST_ANGLE = 0.0;
        public static final double SHOULDER_ANGLE = 1.45;
        public static final double shoulderAvoidTurretAngle = 1.35; //TODO: GOTTA CHANGE
        public static final double wristAvoidTurretAngle = 0.8; //TODO: GOTTA CHANGE
        public static final double DISABLED_WRIST = Math.toRadians(160);
        public static final double SHOULDER_VELOCITY = 1;
        public static final double WRIST_VELOCITY = 1;
      }
    }

    public static class Shoulder {
        public static final double ARM_LENGTH = Units.inchesToMeters(13.38); 
        public static final int MOTOR = 35;
        public static final int ENCODER = 36;
        public static final double ENCODER_OFFSET = Units.radiansToRotations(1.66103);
        public static final double HIGH_BOUND = 1.59;
        public static final double LOW_BOUND = -0.713;
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final double POSITION_ERROR_TOLERANCE = 3;
        public static final double ENCODER_OFFSET_FROM_ZERO = 0.713; 
        public static final double NEUTRAL_VOLTAGE = 0.00;
        public static final double GEAR_RATIO = 52.5;
        public static final double MOI = Units.inchesToMeters(Units.inchesToMeters(Units.lbsToKilograms(421.65))); // lbs sq in -> kg sq m

        public static final double k_G = 0.17;
        public static final double k_P = 4.0; //0.07 * GEAR_RATIO
        public static final double k_D = 0.0;
    }

    public static class Wrist {
        public static final double ARM_LENGTH = Units.inchesToMeters(3.795); // Arbitrary, change me once hardware is finalized
        public static final int MOTOR = 37;
        public static final double ENCODER_OFFSET = 0;//-0.913-0.56; // In radians?????
        public static final double HIGH_BOUND = Math.toRadians(145);
        public static final double LOW_BOUND = -10;
        public static final double ENCODER_OFFSET_FROM_ZERO = 0.0; // In degrees from horizontal as zero (for gravity feedforward calculations)
        public static final double POSITION_ERROR_TOLERANCE = 10;
        public static final double NEUTRAL_VOLTAGE = 0.00;
        // public static final int HALL_EFFECT = 8; //should  be 1 owen  bad
        public static final double GEAR_RATIO = 50.0;
        public static final double MOI = Units.inchesToMeters(Units.inchesToMeters(Units.lbsToKilograms(13.21))); // lbs sq in -> kg sq m
        public static final double MIN_CURRENT_LIMIT = 13;

        public static final double k_G = 0.4;
        public static final double k_P = 0.05 * GEAR_RATIO; //0.15 * GEAR_RATIO
        public static final double k_D = 0.000;
    }

    // Arbitrary, change me once we have a robot
    public static class Roller {
      public static final int MOTOR = 39; //39
      public static final int IR_GATE = 0;
      public static final int carriageIR = 8;
      public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
      public static final double HOLD_SPEED = 0.0;
      public static final double NEUTRAL_SPEED = 0.1;
      public static final double ROLLING_SPEED = 0.3;
      public static final double NEUTRAL_VOLTAGE = 0.0;


      public static enum RollerState {
        INTAKE,
        OUTTAKE,
        NEUTRAL,
        HOLD
      }
    }
  }

  public static class Elevator{
    public static final int CLIMB_RIGHT_MOTOR = 46;
    public static final int CLIMB_LEFT_MOTOR = 45;
    public static final int LEFTSERVO_CHANNEL = 3;//TODO: CHANGE WHEN WE KNOW THE CHANNEL
    public static final int RIGHTSERVO_CHANNEL = 4;
    public static final int HALLEFFECT = 2;
    public static final double SOFT_LIMIT_HEIGHT = Units.inchesToMeters(31.5 + 10); //max is 1.02235
    public static final double HARD_LIMIT_HEIHT = Units.inchesToMeters(31.5);
    public static final double ClimbHeight = Units.inchesToMeters(31.5 + 7);
    //for below constant: 0.0 (going down all the way) would make this redudant so this is to a constant to help us only go down partway
    public static final double HangHeight = Units.inchesToMeters(10); 

    public static final double kP = 35;// 8
    public static final double kD = 0.0;
    public static final double kV = 0.0; //0.2
    public static final double kA = 0.0; //0.2
    public static final double kS = 0.0;
    public static final double kG = 0.15; //0.1
    public static final double maxV = 5;

    public static final double carriageMassKg = 5.443;
    public static final double drumRadiusMeters = 0.0381;
    public static final double gearing = 25.0;

    public static class Sim{
      public static final double kP = 1.0;
      public static final double kD = 0.0;
      public static final double kV = 0.5;
      public static final double kA = 0.5;
      public static final double kS = 0.0;
      public static final double kG = 0.1;
    }
  }

  public static class ShooterFlywheelConstants{
    public static final int kLeftMotorPort = 51;
    public static final int kRightMotorPort = 52;
    public static final double kShooterGearing = 1.8;
    public static final double kShooterMaxRPM = 6380;

    public static final int kBeamBreakPort = 3;
    public static final int kFeederBeamBreakPort = 15; //TODO: CHANGE
    public static final double tolerance = 10;
    public static final double kP = 0.003;
    public static final double kD = 0;
    public static final double kF = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kVoltageMax = 10;
    public static final double height = Units.inchesToMeters(24); //TODO: CHANGE ACCORDING TO HARDWARE
     public class Sim{
      public static final double kP = 0.8;
      public static final double kD = 0.01;
      public static final double kF = 0.01;
      public static final double kV = 0.0;
      public static final double kVP = 0.0;
      public static final double kA = 0;
    }
    public static final Pose2d ampside = new Pose2d(0.7863951921463013, 6.658573627471924, new Rotation2d(1.0448986674864889));
    public static final Pose2d sourceSide = new Pose2d(0.766904354095459, 4.417128086090088, new Rotation2d(-1.0598467586668474));
    public static final Pose2d subwoofer = new Pose2d(1.410, 5.548, new Rotation2d(0.0));
    public static final Pose2d blueline = new Pose2d(6.302, 7.809, new Rotation2d(0.0));
    public static final Pose2d bluelineinner328 = new Pose2d(5.427524566650391, 6.131943225860596, new Rotation2d(0));
    public static final Pose2d wingmidline = new Pose2d(6.302, 7.7695512771606445, new Rotation2d(0.0));
    public static final Pose2d podium = new Pose2d(2.658, 4.125, new Rotation2d(0.0));

    public static final Pose2d feederSource = new Pose2d(9.927596092224121, 0.7333604693412781, new Rotation2d(0.0));

    public static final Pose2d sourceAutoEnd = new Pose2d(2.4820973873138428, 3.4425861835479736, new Rotation2d(-0.7853981633974483));
  }

  public static class ShooterFeederConstants{
    public static final int kMotorPort = 54;

    public static final double feedVoltage = 0;
  }
  public static class ShooterPivotConstants{
    public static final int kMotorPort = 55;
    public static final double kMotorGearing = 60.0/16.0 * 36.0/20.0 * 110.0/10.0;

    public static final int kEncoderPort = 56;
    public static final double kEncoderOffset = -0.294 +200.0/360 - Units.degreesToRotations(58.6 + (58.6 - 47.145)) + Units.degreesToRotations(5);

    public static final double kP = 2; //5.3
    public static final double kD = 0.5; //0.0, .75
    public static final double kV = Units.degreesToRadians(37); //37 degrees per second per volt

    public static final double kLowerBound = 0.237;//Units.degreesToRadians(14);
    public static final double kHigherBound = 1.136;//Units.degreesToRadians(58);

    public class Sim{
      public static final double kP = 12;
      public static final double kD = 0.01;
      public static final double kV = 0.0;
    }
  }
  public static class TurretConstants{
    public static final int kMotorPort = 53;
    public static final double kMotorGearing = 60.0;

    public static final int kEncoderPort = 57;

    public static final double kEncoderOffset = Units.degreesToRotations(178.9-143.8763);
    public static final double kLowerBound = -1;
    public static final double kHigherBound = 1;

    public static final double kP = 9.75;
    public static final double kD = 0.5; //0.25
    public static final double kV = Units.degreesToRadians(50); // 50 degrees per second per volt
    public class Sim{
      public static final double kP = 1;
      public static final double kD = 0.00;
      public static final double kV = 0.0;
    }
  }
  public static class Candles{
    public static final int led = 61;
  }
}