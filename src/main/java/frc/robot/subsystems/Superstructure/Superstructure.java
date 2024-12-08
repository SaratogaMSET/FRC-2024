package frc.robot.subsystems.Superstructure;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Superstructure.Intake.IntakeSubsystem;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/*In robot container,
commandSetGoal from the Superstructure takes a parameter whcih is a state/goal from the Goal enum in Superstructure
and sets that to the desired goal in the Superstrucutre.

In the Superstrucutre periodic, the desired goal is then set to the current goal
and a switchcase calls the setGoal methods of the Intake and Elevator for the respective goal.

The set goal method of the subystem moves the intake/elevator using methods in the respective subsystem
and keeps moving the subsytem until it reaches the setpoint.

In the periodic fucntion of the subsystem,
it checks if the current poisiton equals the goal position using the atGoal method
(the goal position is set in the periodic of the subsystem and takes values from the subystem's Goal enum)
and if it is not, it keeps moving the subsytem.*/

public class Superstructure extends SubsystemBase {
  public enum Goal {
    SHOOTING,
    AMP,
    GROUND_INTAKE,
    STOWED,
    HANG_UP(true),
    HANG_DOWN(true),
    EXTAKING,
    TRAP(true),
    RESET_TRAP(true),
    SOURCE,
    RESET_WRIST;

    private final boolean climbingGoal;

    Goal() {
      this.climbingGoal = false;
    }

    Goal(boolean climbingGoal) {
      this.climbingGoal = climbingGoal;
    }

    public boolean isClimbingGoal() {
      return climbingGoal;
    }
  }

  @Getter private Goal currentGoal = Goal.STOWED;
  @Getter private Goal desiredGoal = Goal.STOWED;

  private final ElevatorSubsystem elevator;
  private final IntakeSubsystem intake;

  public Superstructure(ElevatorSubsystem elevator, IntakeSubsystem intake) {
    this.elevator = elevator;
    this.intake = intake;

    setDefaultCommand(
        new InstantCommand(
            () -> {
              currentGoal = Goal.STOWED;
              desiredGoal = Goal.STOWED;

              intake.setGoal(IntakeSubsystem.Goal.STOWED);
              elevator.setGoal(ElevatorSubsystem.Goal.STOWED);
            },
            this));
  }

  public void setDesiredGoal(Goal goal) {
    desiredGoal = goal;
  }

  public Command commandSetGoal(Goal goal) {
    return Commands.run(() -> setDesiredGoal(goal), this);
  }

  public void periodic() {
    if (DriverStation.isDisabled()) {
      intake.setVoltages(0, 0);
      elevator.setVoltage(0, 0);
    }

    currentGoal = desiredGoal;

    // If the climber is not retracted and the current goal is not a climbing goal, it needs to set
    // the current goal to the reset climb goal.

    switch (currentGoal) {
      case STOWED -> {
        intake.setGoal(IntakeSubsystem.Goal.STOWED);
        elevator.setGoal(ElevatorSubsystem.Goal.STOWED);
      }
      case SHOOTING -> {
        intake.setGoal(IntakeSubsystem.Goal.SHOOTING);
        elevator.setGoal(ElevatorSubsystem.Goal.SHOOTING);
      }
      case AMP -> {
        intake.setGoal(IntakeSubsystem.Goal.AMP);
        elevator.setGoal(ElevatorSubsystem.Goal.AMP);
      }
      case GROUND_INTAKE -> {
        intake.setGoal(IntakeSubsystem.Goal.GROUND_INTAKE);
        elevator.setGoal(ElevatorSubsystem.Goal.GROUND_INTAKE);
      }
      case HANG_UP -> {
        intake.setGoal(IntakeSubsystem.Goal.HANG_UP);
        elevator.setGoal(ElevatorSubsystem.Goal.HANG_UP);
      }
      case HANG_DOWN -> {
        intake.setGoal(IntakeSubsystem.Goal.HANG_DOWN);
        elevator.setGoal(ElevatorSubsystem.Goal.HANG_DOWN);
      }
      case EXTAKING -> {
        intake.setGoal(IntakeSubsystem.Goal.EXTAKING);
        elevator.setGoal(ElevatorSubsystem.Goal.EXTAKING);
      }
      case TRAP -> {
        intake.setGoal(IntakeSubsystem.Goal.TRAP);
        elevator.setGoal(ElevatorSubsystem.Goal.TRAP);
      }
      case RESET_TRAP -> {
        intake.setGoal(IntakeSubsystem.Goal.RESET_TRAP);
        elevator.setGoal(ElevatorSubsystem.Goal.RESET_TRAP);
      }
      case SOURCE -> {
        intake.setGoal(IntakeSubsystem.Goal.SOURCE);
        elevator.setGoal(ElevatorSubsystem.Goal.SOURCE);
      }
      case RESET_WRIST -> {
        intake.setGoal(IntakeSubsystem.Goal.RESET_WRIST);
        elevator.setGoal(ElevatorSubsystem.Goal.RESET_WRIST);
      }
    }

    elevator.periodic();
    intake.periodic();

    Logger.recordOutput("Superstructure/GoalState", desiredGoal);
    Logger.recordOutput("Superstructure/CurrentState", currentGoal);
  }

  @AutoLogOutput(key = "Superstructure/AtArmGoal")
  public boolean atArmGoal() {
    return currentGoal == desiredGoal && intake.atGoal();
  }

  public boolean atGoal() {
    return currentGoal == desiredGoal && intake.atGoal() && elevator.atGoal();
  }
}
