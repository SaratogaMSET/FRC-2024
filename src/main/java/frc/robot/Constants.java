package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;

public class Constants {

    public static final Mode currentMode = Mode.REAL; // This doesn't do anything.
    public static RobotType robot = RobotType.ROBOT_2024C;

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
        public static final Transform3d robotToCam14 = new Transform3d(new Translation3d(Units.inchesToMeters(9.75), -Units.inchesToMeters(13.5), Units.inchesToMeters(10.5)),
            new Rotation3d(Math.toRadians(20),0,Math.toRadians(135))); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d robotToCam11 = new Transform3d(new Translation3d(-Units.inchesToMeters(1), -Units.inchesToMeters(13.5), Units.inchesToMeters(11.3)),
            new Rotation3d(Math.toRadians(20) ,0,Math.toRadians(45))); //Cam mounted facing forward, 3 forward of center,15 inches left of center, 17 up from center.
        public static final Matrix<N3, N1> stateSTD = VecBuilder.fill(0.23, 0.19, 0.005);
        public static final Matrix<N3, N1> visDataSTD = VecBuilder.fill(0.77, 0.81, 0.995);

        public static final double ALIGNMENT_ALLOWED_TOLERANCE_TRANSLATIONAL = 0.1; // meters
        public static final double ALIGNMENT_ALLOWED_TOLERANCE_ROTATIONAL = 0.122; // radians

        public static double xyCoeff = 0.01; // TODO: FIX, THESE VALUES ARE NOT TUNED
        public static double rotationCoeff = 0.01; // TODO: FIX, THESE VALUES ARE NOT TUNED
    } 

  public static class Intake {  // Waiting for hardware (Akash) with angles to return from Tahoe
    public static class DesiredStates {
      public static class Ground {
        public static final double LOWER_MOTION_WRIST_ANGLE = -30;
        public static final double UPPER_MOTION_WRIST_ANGLE = -10;
        public static final double LOWER_MOTION_SHOULDER_ANGLE = 0;
        public static final double UPPER_MOTION_SHOULDER_ANGLE = 45;
        public static final double SHOULDER_VELOCITY = 1;
        public static final double WRIST_VELOCITY = 1;
      }

      public static class Amp {
        public static final double WRIST_ANGLE = 60;
        public static final double SHOULDER_ANGLE = 80;
        public static final double SHOULDER_VELOCITY = 1;
        public static final double WRIST_VELOCITY = 1;
      }

      public static class Trap {
        public static final double WRIST_ANGLE = 139;
        public static final double SHOULDER_ANGLE = 160;
        public static final double SHOULDER_VELOCITY = 1;
        public static final double WRIST_VELOCITY = 1;
      }

      public static class Source {
        public static final double WRIST_ANGLE = 60;
        public static final double SHOULDER_ANGLE = 90;
        public static final double SHOULDER_VELOCITY = 1;
        public static final double WRIST_VELOCITY = 1;
      }

      public static class Neutral {
        public static final double WRIST_ANGLE = 60;
        public static final double SHOULDER_ANGLE = 83.11;
        public static final double SHOULDER_VELOCITY = 1;
        public static final double WRIST_VELOCITY = 1;
        public static final double DISABLED_WRIST = 160;
      }

      public static enum ArmStates {
        GROUND_DEPLOY,
        NEUTRAL,
        AMP,
        SOURCE,
        TRAP,
        MANUAL
      }
    }

    public static class Shoulder {
        public static final double ARM_LENGTH = Units.inchesToMeters(13.38); 
        public static final int MOTOR = 1;
        public static final int ENCODER = 3;
        public static final double ENCODER_OFFSET = 0.0; // In rotations
        public static final double HIGH_BOUND = 180;
        public static final double LOW_BOUND = 0;
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final double ENCODER_OFFSET_FROM_ZERO = 90.0; // In degrees from horizontal as zero (for gravity feedforward calculations)
        public static final double POSITION_ERROR_TOLERANCE = 0;
        public static final double NEUTRAL_VOLTAGE = 0.01;
        public static final double GEAR_RATIO = 52.5;
        public static final double MOI = Units.inchesToMeters(Units.inchesToMeters(Units.lbsToKilograms(421.65))); // lbs sq in -> kg sq m

      public static class ControlsConstants {
        public static final double k_G = 0.1;
        public static final double k_P = 1.0;
        public static final double k_D = 0.000;
        public static final double k_I = 0.000; 
      }
    }

    public static class Wrist {
        public static final double ARM_LENGTH = Units.inchesToMeters(3.795); // Arbitrary, change me once hardware is finalized
        public static final int MOTOR = 13;
        public static final int ENCODER = 10;
        public static final double ENCODER_OFFSET = 0; // In rotations
        public static final double HIGH_BOUND = 180;
        public static final double LOW_BOUND = -45;
        public static final double ENCODER_OFFSET_FROM_ZERO = 0; // In degrees from horizontal as zero (for gravity feedforward calculations)
        public static final double POSITION_ERROR_TOLERANCE = 0;
        public static final double NEUTRAL_VOLTAGE = 0.01;
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final int HALL_EFFECT = 12;
        public static final double GEAR_RATIO = 50.0;
        public static final double MOI = Units.inchesToMeters(Units.inchesToMeters(Units.lbsToKilograms(13.21))); // lbs sq in -> kg sq m

      public static class ControlsConstants {
        public static final double k_G = 0.001;
        public static final double k_P = 1.0;
        public static final double k_D = 0.000;
        public static final double k_I = 0.000; 
      }
    }

    // Arbitrary, change me once we have a robot
    public static class Roller {
      public static final int MOTOR = 5;
      public static final int ENTER_IR_GATE = 6;
      public static final int EXIT_IR_GATE = 21;
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

  public static class ElevatorConstants{
    public static final int CLIMB_RIGHT_MOTOR = 20;
    public static final int CLIMB_LEFT_MOTOR = 21;
    public static final int HALLEFFECT = 22;
    public static final double SOFT_LIMIT_HEIGHT = 31.5; //max is 1.02235
    public static final double HARD_LIMIT_HEIHT = Units.inchesToMeters(31.5);

    public static final double kP = 0.5;
    public static final double kD = 0.0;
    public static final double kV = 0.2;
    public static final double kA = 0.2;
    public static final double kS = 0.0;
    public static final double kG = 0.1;
    public static final double maxV = 10;

    public static final double carriageMassKg = 5.443;
    public static final double drumRadiusMeters = 0.0381;
    public static final double gearing = 25.0;

    public static class Sim{
      public static final double kP = 1;
      public static final double kD = 0.0;
      public static final double kV = 0.5;
      public static final double kA = 0.5;
      public static final double kS = 0.0;
      public static final double kG = 0.5;
    }
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
