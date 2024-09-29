package frc.robot.subsystems.Intake.Shoulder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.Intake.DesiredStates;
import frc.robot.Constants.Intake.Shoulder;

public class ShoulderIOSim implements ShoulderIO {
  private double inputVoltage = 0.0;
  SingleJointedArmSim shoulder =
      new SingleJointedArmSim(
          DCMotor.getFalcon500(1),
          Shoulder.GEAR_RATIO,
          Shoulder.MOI,
          Shoulder.ARM_LENGTH,
          Shoulder.LOW_BOUND,
          Shoulder.HIGH_BOUND,
          true,
          DesiredStates.Neutral.SHOULDER_ANGLE);

  @Override
  /** Updates inputs for shoulder voltage, current and angle in degrees and angleVel */
  public void updateInputs(ShoulderIOInputs inputs) {
    inputs.shoulderRads = shoulder.getAngleRads();
    inputs.shoulderRadPerSecs = shoulder.getVelocityRadPerSec();
    inputs.shoulderCurrent = shoulder.getCurrentDrawAmps();
    inputs.shoulderVoltage = inputVoltage;
    shoulder.update(0.02);
  }

  @Override
  /** Sets shoulder voltage */
  public void setVoltage(double voltage) {
    shoulder.setInputVoltage(voltage);
    inputVoltage = voltage;
  }

  @Override
  /** Sets shoulder to a specific state based on angle (converts to radians) and velocity */
  public void setAngle(double angle, double velocity) {
    shoulder.setState(angle, velocity);
  }
}
