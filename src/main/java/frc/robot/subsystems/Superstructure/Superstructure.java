package frc.robot.subsystems.Superstructure;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Superstructure.Intake.IntakeSubsystem;
import lombok.Getter;
import frc.robot.subsystems.Superstructure.Elevator.ElevatorSubsystem;

public class Superstructure extends SubsystemBase {
  public enum Goal {
    SHOOTING,
    AMP,
    GROUND_INTAKE,
    STOWED,
    HANG_UP(true),
    HANG_DOWN(true),
    EXTAKING,
    TRAP (true),
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

 public void setDesiredGoal(Goal desiredGoal) {
    this.desiredGoal = desiredGoal;
 }


  private final ElevatorSubsystem elevator;
  private final IntakeSubsystem intake;

  public Superstructure(ElevatorSubsystem elevator, IntakeSubsystem intake) {
    this.elevator = elevator;
    this.intake = intake;
    
    setDefaultCommand(new InstantCommand(() -> {
        intake.setGoal(IntakeSubsystem.Goal.STOWED);
        elevator.setGoal(ElevatorSubsystem.Goal.STOWED);
      }));
  }

  private void setGoal(Goal goal) {
    desiredGoal = goal;
  }

  public void periodic() {
    if (DriverStation.isDisabled()) {
      intake.setVoltages(0,0);
      elevator.setVoltage(0,0);
    }

    //If the climber is not retracted and the current goal is not a climbing goal, it needs to set the current goal to the reset climb goal.

    switch (currentGoal) {
      case STOWED -> {
        setGoal(Goal.STOWED);
        intake.setGoal(IntakeSubsystem.Goal.STOWED);
        elevator.setGoal(ElevatorSubsystem.Goal.STOWED);
      }
      case SHOOTING -> {
        setGoal(Goal.SHOOTING);
        intake.setGoal(IntakeSubsystem.Goal.SHOOTING);
        elevator.setGoal(ElevatorSubsystem.Goal.SHOOTING);
      }
      case AMP -> {
        setGoal(Goal.AMP);
        intake.setGoal(IntakeSubsystem.Goal.AMP);
        elevator.setGoal(ElevatorSubsystem.Goal.AMP);
      }
      case GROUND_INTAKE -> {
        setGoal(Goal.GROUND_INTAKE);
        intake.setGoal(IntakeSubsystem.Goal.GROUND_INTAKE);
        elevator.setGoal(ElevatorSubsystem.Goal.GROUND_INTAKE);
      }
      case HANG_UP -> {
        setGoal(Goal.HANG_UP);
        intake.setGoal(IntakeSubsystem.Goal.HANG_UP);
        elevator.setGoal(ElevatorSubsystem.Goal.HANG_UP);
      }
      case HANG_DOWN -> {
        setGoal(Goal.HANG_DOWN);
        intake.setGoal(IntakeSubsystem.Goal.HANG_DOWN);
        elevator.setGoal(ElevatorSubsystem.Goal.HANG_DOWN);
      }
      case EXTAKING -> {
        setGoal(Goal.EXTAKING);
        intake.setGoal(IntakeSubsystem.Goal.EXTAKING);
        elevator.setGoal(ElevatorSubsystem.Goal.EXTAKING);
      }
      case TRAP -> {
        setGoal(Goal.TRAP);
        intake.setGoal(IntakeSubsystem.Goal.TRAP);
        elevator.setGoal(ElevatorSubsystem.Goal.TRAP);
      }
      case RESET_TRAP -> {
        setGoal(Goal.RESET_TRAP);
        intake.setGoal(IntakeSubsystem.Goal.RESET_TRAP);
        elevator.setGoal(ElevatorSubsystem.Goal.RESET_TRAP);
      }
      case SOURCE -> {
        setGoal(Goal.SOURCE);
        intake.setGoal(IntakeSubsystem.Goal.SOURCE);
        elevator.setGoal(ElevatorSubsystem.Goal.SOURCE);
      }
      case RESET_WRIST -> {
        setGoal(Goal.RESET_WRIST);
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


}