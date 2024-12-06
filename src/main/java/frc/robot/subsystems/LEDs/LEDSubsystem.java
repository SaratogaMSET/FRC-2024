package frc.robot.subsystems.LEDs;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Candles;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {

  // led.animate(new FireAnimation(1.0, 0.75, Candles.STRIP_COUNT, 1.0, 0.3, false, 8), 0);

  public enum State {
    SHOOTING_W_VISION,
    SHOOTING_WOUT_VISION,
    NORMAL,
    HAS_NOTE,
  }

  private State state = State.NORMAL;

  public CANdle led = new CANdle(Candles.led, Constants.CANBus);

  private final FireAnimation fireAnimation =
      new FireAnimation(1.0, 0.75, Candles.STRIP_COUNT, 1.0, 0.3, false, Candles.STRIP_START_IDX);
  private final SingleFadeAnimation powderBlueAnimation =
      new SingleFadeAnimation(
          176, 224, 230, 0, 0.03, Candles.STRIP_COUNT, Candles.STRIP_START_IDX); // Powder blue

  private final RainbowAnimation rainbowAnimation =
      new RainbowAnimation(1, 1, Candles.STRIP_COUNT, false, Candles.STRIP_START_IDX);

  private final SingleFadeAnimation purpleAnimation = createSingleFadeAnimation(128, 0, 128, 0);

  private final SingleFadeAnimation orangeAnimation = createSingleFadeAnimation(255, 10, 0, 0);

  private final LarsonAnimation blueAllianceAnimation = createLarsonAnimation(4, 2, 155, 0);

  private final LarsonAnimation redAllianceAnimation = createLarsonAnimation(199, 21, 133, 0);

  private final ColorFlowAnimation orangeFlowAnimation = createColorFlowAnimation(255, 10, 0, 0);

  public LEDSubsystem() {
    CANdleConfiguration candleConfiguration = new CANdleConfiguration();
    candleConfiguration.statusLedOffWhenActive = true;
    candleConfiguration.disableWhenLOS = false;
    candleConfiguration.stripType = LEDStripType.RGB;
    candleConfiguration.brightnessScalar = 1.0;
    candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
    led.configAllSettings(candleConfiguration);
  }

  @Override
  public void periodic() {
    double startTime = Timer.getFPGATimestamp();

    switch (state) {
      case NORMAL:
        if (DriverStation.isDSAttached()) {
          if (DriverStation.isAutonomousEnabled()) {
            animate(fireAnimation);
          } else {
            animate(powderBlueAnimation);
          }
        } else {
          allianceAnim();
        }
        break;
      case SHOOTING_W_VISION:
        animate(purpleAnimation);
        break;
      case SHOOTING_WOUT_VISION:
        animate(rainbowAnimation);
        break;
      case HAS_NOTE:
        animate(orangeAnimation);
        break;
    }

    double runtimeMS = (Timer.getFPGATimestamp() - startTime) * 1000;
    Logger.recordOutput("LED Periodic Runtime", runtimeMS);
    Logger.recordOutput("LED State", state.toString());
  }

  public void clearAnimation() {
    led.clearAnimation(0);
  }

  public void animate(Animation animation) {
    led.animate(animation, 0);
  }

  public LarsonAnimation createLarsonAnimation(int r, int g, int b, int w) {
    return new LarsonAnimation(
        r,
        g,
        b,
        w,
        0.03,
        Candles.STRIP_COUNT,
        LarsonAnimation.BounceMode.Center,
        7,
        Candles.STRIP_START_IDX);
  }

  public ColorFlowAnimation createColorFlowAnimation(int r, int g, int b, int w) {
    return new ColorFlowAnimation(
        r,
        g,
        b,
        w,
        1,
        Candles.STRIP_COUNT,
        ColorFlowAnimation.Direction.Forward,
        Candles.STRIP_START_IDX);
  }

  public SingleFadeAnimation createSingleFadeAnimation(int r, int g, int b, int w) {
    return new SingleFadeAnimation(r, g, b, w, 0.03, Candles.STRIP_COUNT, Candles.STRIP_START_IDX);
  }

  private void allianceAnim() {
    if (DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red) {
      animate(redAllianceAnimation);
    } else {
      animate(blueAllianceAnimation);
    }
  }

  public void setState(State state) {
    this.state = state;
  }

  public State getState() {
    return state;
  }

  public Command setStateCommand(State state) {
    return runOnce(() -> setState(state)).ignoringDisable(true);
  }
}
