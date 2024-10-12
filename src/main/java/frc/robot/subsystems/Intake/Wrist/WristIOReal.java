package frc.robot.subsystems.Intake.Wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Intake.Wrist;
import org.littletonrobotics.junction.Logger;

public class WristIOReal implements WristIO {

  /* PLEASE NEVER CALL HALLEFFECT.GET(). YOU WOULD BE GREIFING. PLEASE CALL THE CLASS'S GETTER. THANK YOU */
  static boolean previousCurrentLimit = false;

  public WristIOReal() {
    for (int i = 0; i < 30; i++) {
      motor.setSmartCurrentLimit(20);
      motor.setInverted(true);
    }
  }

  @Override
  /** Updates inputs for wrist angle in degrees and the status of the hall effect sensor */
  public void updateInputs(WristIOInputs inputs) {
    inputs.wristHallEffect =
        getCurrentLimitTripped(); // Returns true if the sensor senses the wrist!!!!!
    inputs.wristRads = (2 * Math.PI * (motor.getEncoder().getPosition() / Wrist.GEAR_RATIO));
    inputs.wristCurrent = motor.getOutputCurrent(); // LOl
    inputs.wristVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.wristRotations = motor.getEncoder().getPosition();
  }

  @Override
  public void setWristPosition(double angle) {
    motor.getEncoder().setPosition(angle);
  }

  @Override
  /** Sets wrist voltage */
  public void setVoltage(double voltage) {
    SmartDashboard.putNumber("haha find me 2.0 wrist motor voltage", voltage);
    motor.setVoltage(voltage);
  }

  @Override
  /**
   * Dude what does this do. Resets wrist motor encoder if the wrist has just reached close to the
   * sensor
   */
  public boolean hallEffectReset() {
    boolean test = false;
    if (!previousCurrentLimit
        && getCurrentLimitTripped()) { // If previous = false and current = true, we can reset hall
      // effect.
      // Returns true.
      test = true;
      // double newZeroPos = 0.15 / (2 * Math.PI) * Wrist.GEAR_RATIO;
      // motor.getEncoder().setPosition(0);
      // wristIOInputs.wristDegrees = AcutatorConstants.WRIST_ENCODER_HALL_EFFECT;
    }
    previousCurrentLimit = getCurrentLimitTripped();
    return test;
  }

  @Override
  public void manualHallEffectReset() {
    motor.getEncoder().setPosition(0);
  }

  @Override
  public boolean getCurrentLimitTripped() {
    Logger.recordOutput("WristFunnyOutputCurrent", motor.getOutputCurrent());
    return motor.getOutputCurrent() > Wrist.MIN_CURRENT_LIMIT;
  }
}
