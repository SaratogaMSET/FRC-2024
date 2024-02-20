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

    public static final Mode currentMode = Mode.SIM;
    public static RobotType robot = RobotType.ROBOT_2024P;

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
        public static final Transform3d robotToCam12 = new Transform3d(new Translation3d(-Units.inchesToMeters(3), -Units.inchesToMeters(15), Units.inchesToMeters(17)),
            new Rotation3d(Math.toRadians(45),0,Math.toRadians(-45))); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d robotToCam11 = new Transform3d(new Translation3d(Units.inchesToMeters(3), -Units.inchesToMeters(15), Units.inchesToMeters(12)),
            new Rotation3d(Math.toRadians(20) ,0,Math.toRadians(45))); //Cam mounted facing forward, 3 forward of center,15 inches left of center, 17 up from center.
        public static final Matrix<N3, N1> stateSTD = VecBuilder.fill(0.23, 0.19, 0.005);
        public static final Matrix<N3, N1> visDataSTD = VecBuilder.fill(0.77, 0.81, 0.995);

        public static final double ALIGNMENT_ALLOWED_TOLERANCE_TRANSLATIONAL = 0.1; // meters
        public static final double ALIGNMENT_ALLOWED_TOLERANCE_ROTATIONAL = 0.122; // radians

        public static double xyCoeff = 0.01; // TODO: FIX, THESE VALUES ARE NOT TUNED
        public static double rotationCoeff = 0.01; // TODO: FIX, THESE VALUES ARE NOT TUNED
    } 

  public static class Intake {
    public static class AcutatorConstants {
      public static enum ActuatorState {
        GROUND_DEPLOY,
        NEUTRAL,
        AMP,
        SOURCE,
        TRAP,
        MANUAL
      }
        public static final double SHOULDER_LENGTH = Units.inchesToMeters(13.38); 
        public static final double WRIST_ANGLE = 0.0; // Arbitrary, change me once hardware is finalized
        public static final double WRIST_LENGTH = Units.inchesToMeters(3.795); // Arbitrary, change me once hardware is finalized
        public static final double SHOULDER_ANGLE = 0.0; // Arbitrary, change me once hardware is finalized
        public static final double SPEED = 0.2; // Arbitrary, change me during testing
        public static final double carriageShoulderPivotOffset = Units.inchesToMeters(2);

      public static class GroundNeutralPerimeterConstants {
        public static final double LOWER_MOTION_WRIST_ANGLE = 150;
        public static final double UPPER_MOTION_WRIST_ANGLE = 50;
        public static final double LOWER_MOTION_SHOULDER_ANGLE = -30;
        public static final double UPPER_MOTION_SHOULDER_ANGLE = 40;
        public static final double SHOULDER_POWER_PERCENT = 0.05;
        public static final double WRIST_POWER_PERCENT = 0.05;
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
        public static final double SOURCE_SHOULDER_ANGLE = 100;
      }

      public static class WristControlsConstants {
        public static final double k_G = 0.001;
        public static final double k_P = 1.0;
        public static final double k_D = 0.000;
        public static final double k_I = 0.000; 
      }

      public static class ShoulderControlsConstants {
        public static final double k_G = 0.1;
        public static final double k_P = 1.0;
        public static final double k_D = 0.000;
        public static final double k_I = 0.000; 
      }

      // Arbitrary, change all once we have a robot
      public static final int INTAKE_SHOULDER_MOTOR = 1;
      public static final int INTAKE_WRIST_MOTOR = 13;
      public static final int INTAKE_SHOULDER_ENCODER = 3;
      public static final int INTAKE_WRIST_ENCODER = 10;
      public static final NeutralModeValue ARM_NEUTRAL_MODE = NeutralModeValue.Brake;
      public static final double WRIST_ENCODER_OFFSET = 0; // In rotations
      public static final double SHOULDER_ENCODER_OFFSET = 0; // In rotations
      public static final double WRIST_HIGH_BOUND = 180;
      public static final double WRIST_LOW_BOUND = 0;
      public static final double SHOULDER_HIGH_BOUND = 90;
      public static final double SHOULDER_LOW_BOUND = 0;
      public static final double SHOULDER_ENCODER_OFFSET_FROM_ZERO = 0; // In degrees from horizontal as zero (for gravity feedforward calculations)
      public static final double WRIST_ENCODER_OFFSET_FROM_ZERO = 0; // In degrees from horizontal as zero (for gravity feedforward calculations)
      public static final double SHOULDER_ERROR_TOLERANCE = 0; // In degrees
      public static final double WRIST_ERROR_TOLERANCE = 0; // In degrees
      public static final double NEUTRAL_VOLTAGE = 0.01;
      public static final IdleMode INTAKE_BRAKE_MODE = IdleMode.kBrake;
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
}
