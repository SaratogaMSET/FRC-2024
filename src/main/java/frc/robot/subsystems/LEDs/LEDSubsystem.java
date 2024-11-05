package frc.robot.subsystems.LEDs;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Candles;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {
  public CANdle led = new CANdle(Candles.led, Constants.CANBus);
  private int _r = 0;
  private int _g = 0;
  private int _b = 255;
  private double startTime = Timer.getFPGATimestamp();
  private int num_leds = 10;

  public enum ColorPositions {
    INTAKE_AMP_MODE,
    INTAKE_GROUND_MODE,
    INTAKE_SPEAKER_MODE
  }

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
    Logger.recordOutput("Commands/LEDCurrentCommand", this.getCurrentCommand().getName());
  }

  @Override
  public void simulationPeriodic() {
    Logger.recordOutput("Commands/LEDCurrentCommand", this.getCurrentCommand().getName());
  }

  /**
   * Not to be ran outside subsystem. Need to delete animation first.
   *
   * @param r Red value
   * @param g Green value
   * @param b Blue Value
   * @return Command for linking.
   */
  private Command setColor(int r, int g, int b) {
    return this.runOnce(() -> led.setLEDs(r, g, b, 0, 8, 30));
  }

  /**
   * To be ran outside subsystem. Deletes Animation and Sets True color!
   *
   * @param r Red Value
   * @param g Green Value
   * @param b Blue Value
   * @return COmmand for Linking
   */
  public Command color(int r, int g, int b) {
    return (setColor(r, g, b));
  }

  public Color lerpColor(Color a, Color b, double t) {
    return new Color(
        (b.red - a.red) * t + a.red,
        (b.green - a.green) * t + a.green,
        (b.blue - a.blue) * t + a.blue);
  }

  public Color lerpColors(Color[] colors, double t) {
    t *= colors.length;
    int index = (int) t;
    return lerpColor(colors[index], colors[index + 1], t - index);
  }

  public Color loopColor(Color a, Color b, double t) {
    t *= 2;
    if (t > 1) t = -t + 2;
    return lerpColor(a, b, t);
  }

  public Color loopColors(Color[] colors, double t) {
    t *= 2;
    if (t > 1) t = -t + 2;
    return lerpColors(colors, t);
  }

  public void aquamarineAnimation(double periodicity) {
    Color aqA = new Color(24, 255, 95);
    Color aqB = new Color(0, 157, 255);
    double time = (Timer.getFPGATimestamp() - startTime) / periodicity;
    for (int i = 0; i < num_leds; i++) {
      double percentIndex = i / (num_leds - 1);
      double t = (time + percentIndex) - (int) (time + percentIndex);
      Color gradient = loopColor(aqA, aqB, t);
      led.setLEDs((int) gradient.red, (int) gradient.green, (int) gradient.blue, 0, i, 1);
    }
  }

  public void rosegoldAnimation(double periodicity) {
    Color[] colors =
        new Color[] {
          new Color(192, 0, 138),
          new Color(255, 24, 116),
          new Color(255, 103, 46),
          new Color(255, 173, 61),
          new Color(227, 216, 25)
        };
    double time = (Timer.getFPGATimestamp() - startTime) / periodicity;
    for (int i = 0; i < num_leds; i++) {
      double percentIndex = i / (num_leds - 1);
      double t = (time + percentIndex) - (int) (time + percentIndex);
      Color gradient = loopColors(colors, t);
      led.setLEDs((int) gradient.red, (int) gradient.green, (int) gradient.blue, 0, i, 1);
    }
  }

  public Command rainbowFireAnimation(double periodicity) {
    return Commands.runOnce(
            () -> {
              led.animate(new FireAnimation(1.0, 0.75, 36, 1.0, 0.3, false, 8), 0);
              led.setLEDs(0, 0, 0, 0, 0, 8);
            },
            this)
        .finallyDo(() -> deleteEverything());
  }

  public Command deleteEverything() {
    return Commands.runOnce(() -> led.clearAnimation(0), this).andThen(setColor(0, 0, 0));
  }
}
